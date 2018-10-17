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
#include "fordyca/controller/depth1/sensing_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/ds/perceived_arena_map.hpp"
#include "fordyca/fsm/depth0/stateful_fsm.hpp"
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

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
tasking_initializer::tasking_initializer(
    const controller::block_selection_matrix* bsel_matrix,
    const controller::cache_selection_matrix* csel_matrix,
    controller::saa_subsystem* const saa,
    base_perception_subsystem* const perception)
    : stateful_tasking_initializer(bsel_matrix, saa, perception),
      ER_CLIENT_INIT("fordyca.controller.depth1.tasking_initializer"),
      mc_sel_matrix(csel_matrix) {
      }

tasking_initializer::~tasking_initializer(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
tasking_initializer::tasking_map tasking_initializer::depth1_tasks_create(
    params::depth1::controller_repository* const param_repo) {
  auto* task_params = param_repo->parse_results<ta::task_allocation_params>();
  ER_ASSERT(block_sel_matrix(), "NULL block selection matrix");
  ER_ASSERT(cache_sel_matrix(), "NULL cache selection matrix");
  std::unique_ptr<ta::taskable> collector_fsm =
      rcppsw::make_unique<fsm::depth1::cached_block_to_nest_fsm>(
          cache_sel_matrix(), saa_subsystem(), perception()->map());

  std::unique_ptr<ta::taskable> harvester_fsm =
      rcppsw::make_unique<fsm::depth1::block_to_existing_cache_fsm>(
          block_sel_matrix(),
          mc_sel_matrix,
          saa_subsystem(),
          perception()->map());

  tasks::depth1::collector* collector =
      new tasks::depth1::collector(task_params, std::move(collector_fsm));

  auto harvester =
      new tasks::depth1::harvester(task_params, std::move(harvester_fsm));
  auto generalist = graph()->find_vertex(
      tasks::depth0::foraging_task::kGeneralistName);
  generalist->set_partitionable(true);
  generalist->set_atomic(false);

  graph()->install_tab(tasks::depth0::foraging_task::kGeneralistName,
                       std::vector<ta::polled_task*>({harvester, collector}));
  return tasking_map{{"collector", collector}, {"harvester", harvester}};
} /* depth1_tasks_create() */

void tasking_initializer::depth1_exec_est_init(
    params::depth1::controller_repository* const param_repo,
    const tasking_map& map) {
  auto* task_params = param_repo->parse_results<ta::task_allocation_params>();
  if (task_params->exec_est.seed_enabled) {
    /*
     * Generalist is not partitionable in depth 0 initialization, so this has
     * not been done.
     */
    auto harvester = map.find("harvester")->second;
    auto collector = map.find("collector")->second;
    if (0 == std::rand() % 2) {
      graph()->root_tab()->last_subtask(harvester);
    } else {
      graph()->root_tab()->last_subtask(collector);
    }

    uint h_min = task_params->exec_est.ranges.find("harvester")->second.get_min();
    uint h_max = task_params->exec_est.ranges.find("harvester")->second.get_max();
    uint c_min = task_params->exec_est.ranges.find("collector")->second.get_min();
    uint c_max = task_params->exec_est.ranges.find("collector")->second.get_max();
    ER_INFO("Seeding exec estimate for tasks: '%s'=[%u,%u], '%s'=[%u,%u]",
            harvester->name().c_str(),
            h_min,
            h_max,
            collector->name().c_str(),
            c_min,
            c_max);
    static_cast<ta::polled_task*>(harvester)->exec_estimate_init(h_min, h_max);
    static_cast<ta::polled_task*>(collector)->exec_estimate_init(c_min, c_max);
  }
} /* depth1_exec_est_init() */

std::unique_ptr<ta::bi_tdgraph_executive> tasking_initializer::operator()(
    params::depth1::controller_repository* const param_repo) {
  stateful_tasking_init(param_repo);

  auto map = depth1_tasks_create(param_repo);
  auto* executivep = param_repo->parse_results<ta::task_executive_params>();
  graph()->active_tab_init(executivep->tab_init_method);
  depth1_exec_est_init(param_repo, map);

  auto* execp = param_repo->parse_results<ta::task_executive_params>();
  return rcppsw::make_unique<ta::bi_tdgraph_executive>(execp, graph());
} /* initialize() */

NS_END(depth1, controller, fordyca);
