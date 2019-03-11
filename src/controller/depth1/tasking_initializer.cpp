/**
 * @file tasking_initializer.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
 *
 * This file is part of FORDYCA.
 *
 * FORDYCA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * FORDYCA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * FORDYCA.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/depth1/tasking_initializer.hpp"
#include <vector>

#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/base_perception_subsystem.hpp"
#include "fordyca/controller/dpo_perception_subsystem.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/fsm/depth0/dpo_fsm.hpp"
#include "fordyca/fsm/depth1/block_to_existing_cache_fsm.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/params/depth1/controller_repository.hpp"
#include "fordyca/tasks/depth0/generalist.hpp"
#include "fordyca/tasks/depth1/collector.hpp"
#include "fordyca/tasks/depth1/harvester.hpp"

#include "rcppsw/task_allocation/bi_tdgraph.hpp"
#include "rcppsw/task_allocation/bi_tdgraph_executive.hpp"
#include "rcppsw/task_allocation/task_allocation_params.hpp"
#include "rcppsw/task_allocation/task_executive_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);
using ds::occupancy_grid;
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
tasking_initializer::tasking_initializer(
    const controller::block_sel_matrix* bsel_matrix,
    const controller::cache_sel_matrix* csel_matrix,
    controller::saa_subsystem* const saa,
    base_perception_subsystem* const perception)
    : ER_CLIENT_INIT("fordyca.controller.depth1.tasking_initializer"),
      m_saa(saa),
      m_perception(perception),
      mc_csel_matrix(csel_matrix),
      mc_bsel_matrix(bsel_matrix),
      m_graph(nullptr) {}

tasking_initializer::~tasking_initializer(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
tasking_initializer::tasking_map tasking_initializer::depth1_tasks_create(
    const params::depth1::controller_repository& param_repo) {
  auto* task_params = param_repo.parse_results<ta::task_allocation_params>();
  m_graph = new ta::bi_tdgraph(task_params);
  ER_ASSERT(mc_bsel_matrix, "NULL block selection matrix");
  ER_ASSERT(mc_csel_matrix, "NULL cache selection matrix");

  std::unique_ptr<ta::taskable> generalist_fsm =
      rcppsw::make_unique<fsm::depth0::free_block_to_nest_fsm>(
          mc_bsel_matrix, m_saa, m_perception->dpo_store());
  std::unique_ptr<ta::taskable> collector_fsm =
      rcppsw::make_unique<fsm::depth1::cached_block_to_nest_fsm>(
          cache_sel_matrix(), saa_subsystem(), m_perception->dpo_store());

  std::unique_ptr<ta::taskable> harvester_fsm =
      rcppsw::make_unique<fsm::depth1::block_to_existing_cache_fsm>(
          block_sel_matrix(),
          mc_csel_matrix,
          saa_subsystem(),
          m_perception->dpo_store());

  tasks::depth1::collector* collector =
      new tasks::depth1::collector(task_params, std::move(collector_fsm));

  auto harvester =
      new tasks::depth1::harvester(task_params, std::move(harvester_fsm));
  auto generalist =
      new tasks::depth0::generalist(task_params, std::move(generalist_fsm));
  generalist->set_partitionable(true);
  generalist->set_atomic(false);

  m_graph->set_root(generalist);
  m_graph->install_tab(tasks::depth0::foraging_task::kGeneralistName,
                       std::vector<ta::polled_task*>({harvester, collector}));
  return tasking_map{{"generalist", generalist},
                     {"collector", collector},
                     {"harvester", harvester}};
} /* depth1_tasks_create() */

void tasking_initializer::depth1_exec_est_init(
    const params::depth1::controller_repository& param_repo,
    const tasking_map& map) {
  auto* task_params = param_repo.parse_results<ta::task_allocation_params>();
  if (!task_params->exec_est.seed_enabled) {
    /*
     * Generalist is not partitionable in depth 0 initialization, so this has
     * not been done.
     */
    auto harvester = map.find("harvester")->second;
    auto collector = map.find("collector")->second;
    auto generalist = map.find("generalist")->second;

    /*
     * As part of seeding exec estimates, we set the last executed subtask for a
     * TAB. It's OK to declare/use here as local variables as this code only
     * runs during initialization.
     */
    std::default_random_engine eng;
    std::uniform_int_distribution<> dist(0, 1);

    if (0 == dist(eng)) {
      m_graph->root_tab()->last_subtask(harvester);
    } else {
      m_graph->root_tab()->last_subtask(collector);
    }

    rmath::rangeu g_bounds =
        task_params->exec_est.ranges.find("generalist")->second;

    rmath::rangeu h_bounds =
        task_params->exec_est.ranges.find("harvester")->second;
    rmath::rangeu c_bounds =
        task_params->exec_est.ranges.find("collector")->second;
    ER_INFO("Seeding exec estimate for tasks: '%s'=%s '%s'=%s '%s'=%s",
            generalist->name().c_str(),
            g_bounds.to_str().c_str(),
            harvester->name().c_str(),
            h_bounds.to_str().c_str(),
            collector->name().c_str(),
            c_bounds.to_str().c_str());
    static_cast<ta::polled_task*>(generalist)->exec_estimate_init(g_bounds);
    static_cast<ta::polled_task*>(harvester)->exec_estimate_init(h_bounds);
    static_cast<ta::polled_task*>(collector)->exec_estimate_init(c_bounds);
  }
} /* depth1_exec_est_init() */

std::unique_ptr<ta::bi_tdgraph_executive> tasking_initializer::operator()(
    const params::depth1::controller_repository& param_repo) {
  auto map = depth1_tasks_create(param_repo);
  auto* executivep = param_repo.parse_results<ta::task_executive_params>();
  m_graph->active_tab_init(executivep->tab_init_method);
  depth1_exec_est_init(param_repo, map);

  auto* execp = param_repo.parse_results<ta::task_executive_params>();
  return rcppsw::make_unique<ta::bi_tdgraph_executive>(execp, m_graph);
} /* initialize() */

NS_END(depth1, controller, fordyca);
