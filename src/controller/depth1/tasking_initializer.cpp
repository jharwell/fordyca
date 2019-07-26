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
#include <chrono>
#include <vector>

#include "fordyca/config/depth1/controller_repository.hpp"
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
#include "fordyca/fsm/expstrat/block_factory.hpp"
#include "fordyca/fsm/expstrat/cache_factory.hpp"
#include "fordyca/tasks/depth0/generalist.hpp"
#include "fordyca/tasks/depth1/collector.hpp"
#include "fordyca/tasks/depth1/harvester.hpp"

#include "rcppsw/ta/bi_tdgraph.hpp"
#include "rcppsw/ta/bi_tdgraph_executive.hpp"
#include "rcppsw/ta/config/task_alloc_config.hpp"
#include "rcppsw/ta/config/task_executive_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);

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
      mc_bsel_matrix(bsel_matrix) {}

tasking_initializer::~tasking_initializer(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
tasking_initializer::tasking_map tasking_initializer::depth1_tasks_create(
    const config::depth1::controller_repository& config_repo,
    rta::bi_tdgraph* const graph) {
  auto* task_config = config_repo.config_get<rta::config::task_alloc_config>();
  auto* exp_config = config_repo.config_get<config::exploration_config>();
  fsm::expstrat::block_factory block_factory;
  fsm::expstrat::cache_factory cache_factory;
  fsm::expstrat::base_expstrat::params expbp(mc_csel_matrix,
                                             saa_subsystem(),
                                             m_perception->dpo_store());

  ER_ASSERT(nullptr != mc_bsel_matrix, "NULL block selection matrix");
  ER_ASSERT(nullptr != mc_csel_matrix, "NULL cache selection matrix");

  auto generalist_fsm = std::make_unique<fsm::depth0::free_block_to_nest_fsm>(
      mc_bsel_matrix,
      m_saa,
      m_perception->dpo_store(),
      block_factory.create(exp_config->block_strategy, &expbp));
  auto collector_fsm = std::make_unique<fsm::depth1::cached_block_to_nest_fsm>(
      cache_sel_matrix(),
      saa_subsystem(),
      m_perception->dpo_store(),
      cache_factory.create(exp_config->cache_strategy, &expbp));

  fsm::depth1::block_to_existing_cache_fsm::params harvestorp = {
      .bsel_matrix = block_sel_matrix(),
      .csel_matrix = mc_csel_matrix,
      .saa = saa_subsystem(),
      .store = m_perception->dpo_store(),
      .exp_config = *exp_config};

  auto harvester_fsm =
      std::make_unique<fsm::depth1::block_to_existing_cache_fsm>(&harvestorp);

  auto collector =
      std::make_unique<tasks::depth1::collector>(task_config,
                                                 std::move(collector_fsm));

  auto harvester =
      std::make_unique<tasks::depth1::harvester>(task_config,
                                                 std::move(harvester_fsm));
  auto generalist =
      std::make_unique<tasks::depth0::generalist>(task_config,
                                                  std::move(generalist_fsm));
  generalist->set_partitionable(true);
  generalist->set_atomic(false);

  graph->set_root(std::move(generalist));
  rta::bi_tdgraph::vertex_vector children;
  children.push_back(std::move(harvester));
  children.push_back(std::move(collector));
  graph->install_tab(tasks::depth0::foraging_task::kGeneralistName,
                     std::move(children));
  return tasking_map{
      {"generalist",
       graph->find_vertex(tasks::depth0::foraging_task::kGeneralistName)},
      {"collector",
       graph->find_vertex(tasks::depth1::foraging_task::kCollectorName)},
      {"harvester",
       graph->find_vertex(tasks::depth1::foraging_task::kHarvesterName)}};
} /* depth1_tasks_create() */

void tasking_initializer::depth1_exec_est_init(
    const config::depth1::controller_repository& config_repo,
    const tasking_map& map,
    rta::bi_tdgraph* const graph) {
  auto* task_config = config_repo.config_get<rta::config::task_alloc_config>();
  if (!task_config->exec_est.seed_enabled) {
    return;
  }
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
  std::default_random_engine eng(
      std::chrono::system_clock::now().time_since_epoch().count());
  std::uniform_int_distribution<> dist(0, 1);

  if (0 == dist(eng)) {
    graph->root_tab()->last_subtask(harvester);
  } else {
    graph->root_tab()->last_subtask(collector);
  }

  rmath::rangeu g_bounds =
      task_config->exec_est.ranges.find("generalist")->second;

  rmath::rangeu h_bounds =
      task_config->exec_est.ranges.find("harvester")->second;
  rmath::rangeu c_bounds =
      task_config->exec_est.ranges.find("collector")->second;
  ER_INFO("Seeding exec estimate for tasks: '%s'=%s '%s'=%s '%s'=%s",
          generalist->name().c_str(),
          g_bounds.to_str().c_str(),
          harvester->name().c_str(),
          h_bounds.to_str().c_str(),
          collector->name().c_str(),
          c_bounds.to_str().c_str());
  generalist->exec_estimate_init(g_bounds);
  harvester->exec_estimate_init(h_bounds);
  collector->exec_estimate_init(c_bounds);
} /* depth1_exec_est_init() */

std::unique_ptr<rta::bi_tdgraph_executive> tasking_initializer::operator()(
    const config::depth1::controller_repository& config_repo) {
  auto* task_config = config_repo.config_get<rta::config::task_alloc_config>();
  auto graph = std::make_unique<rta::bi_tdgraph>(task_config);
  auto map = depth1_tasks_create(config_repo, graph.get());
  const auto* execp =
      std::make_unique<rta::config::task_executive_config>().get();

  /* can be omitted if the user wants the default values */
  if (nullptr != execp) {
    execp = config_repo.config_get<rta::config::task_executive_config>();
  }

  graph->active_tab_init(execp->tab_init_method);
  depth1_exec_est_init(config_repo, map, graph.get());

  return std::make_unique<rta::bi_tdgraph_executive>(execp, std::move(graph));
} /* initialize() */

NS_END(depth1, controller, fordyca);
