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
#include "fordyca/controller/depth2/tasking_initializer.hpp"
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/base_perception_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/fsm/depth2/block_to_cache_site_fsm.hpp"
#include "fordyca/fsm/depth2/block_to_new_cache_fsm.hpp"
#include "fordyca/fsm/depth2/cache_transferer_fsm.hpp"
#include "fordyca/params/depth2/controller_repository.hpp"
#include "fordyca/tasks/depth1/collector.hpp"
#include "fordyca/tasks/depth2/cache_collector.hpp"
#include "fordyca/tasks/depth2/cache_finisher.hpp"
#include "fordyca/tasks/depth2/cache_starter.hpp"
#include "fordyca/tasks/depth2/cache_transferer.hpp"

#include "rcppsw/ta/bi_tdgraph.hpp"
#include "rcppsw/ta/bi_tdgraph_executive.hpp"
#include "rcppsw/ta/task_alloc_params.hpp"
#include "rcppsw/ta/task_executive_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
tasking_initializer::tasking_initializer(
    const controller::block_sel_matrix* bsel_matrix,
    const controller::cache_sel_matrix* csel_matrix,
    controller::saa_subsystem* const saa,
    base_perception_subsystem* const perception)
    : depth1::tasking_initializer(bsel_matrix, csel_matrix, saa, perception),
      ER_CLIENT_INIT("fordyca.controller.depth2.tasking_initializer") {}

tasking_initializer::~tasking_initializer(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
tasking_initializer::tasking_map tasking_initializer::depth2_tasks_create(
    const params::depth2::controller_repository& param_repo,
    rta::bi_tdgraph* const graph) {
  auto* task_params = param_repo.parse_results<rta::task_alloc_params>();

  std::unique_ptr<rta::taskable> cache_starter_fsm =
      rcppsw::make_unique<fsm::depth2::block_to_cache_site_fsm>(
          block_sel_matrix(),
          cache_sel_matrix(),
          saa_subsystem(),
          perception()->dpo_store());

  std::unique_ptr<rta::taskable> cache_finisher_fsm =
      rcppsw::make_unique<fsm::depth2::block_to_new_cache_fsm>(
          block_sel_matrix(),
          cache_sel_matrix(),
          saa_subsystem(),
          perception()->dpo_store());

  std::unique_ptr<rta::taskable> cache_transferer_fsm =
      rcppsw::make_unique<fsm::depth2::cache_transferer_fsm>(
          cache_sel_matrix(), saa_subsystem(), perception()->dpo_store());
  std::unique_ptr<rta::taskable> cache_collector_fsm =
      rcppsw::make_unique<fsm::depth1::cached_block_to_nest_fsm>(
          cache_sel_matrix(), saa_subsystem(), perception()->dpo_store());

  auto cache_starter = rcppsw::make_unique<tasks::depth2::cache_starter>(
      task_params, std::move(cache_starter_fsm));
  auto cache_finisher = rcppsw::make_unique<tasks::depth2::cache_finisher>(
      task_params, std::move(cache_finisher_fsm));
  auto cache_transferer = rcppsw::make_unique<tasks::depth2::cache_transferer>(
      task_params, std::move(cache_transferer_fsm));
  auto cache_collector = rcppsw::make_unique<tasks::depth2::cache_collector>(
      task_params, std::move(cache_collector_fsm));

  auto collector =
      graph->find_vertex(tasks::depth1::foraging_task::kCollectorName);
  auto harvester =
      graph->find_vertex(tasks::depth1::foraging_task::kHarvesterName);

  collector->set_partitionable(true);
  collector->set_atomic(false);
  harvester->set_partitionable(true);
  harvester->set_atomic(false);
  rta::bi_tdgraph::vertex_vector children1;
  children1.push_back(std::move(cache_starter));
  children1.push_back(std::move(cache_finisher));
  rta::bi_tdgraph::vertex_vector children2;
  children2.push_back(std::move(cache_transferer));
  children2.push_back(std::move(cache_collector));

  graph->install_tab(tasks::depth1::foraging_task::kHarvesterName,
                     std::move(children1));
  graph->install_tab(tasks::depth1::foraging_task::kCollectorName,
                     std::move(children2));
  return tasking_map{
      {"cache_starter",
       graph->find_vertex(tasks::depth2::foraging_task::kCacheStarterName)},
      {"cache_finisher",
       graph->find_vertex(tasks::depth2::foraging_task::kCacheFinisherName)},
      {"cache_transferer",
       graph->find_vertex(tasks::depth2::foraging_task::kCacheTransfererName)},
      {"cache_collector",
       graph->find_vertex(tasks::depth2::foraging_task::kCacheCollectorName)}};
} /* depth2_tasks_create() */

void tasking_initializer::depth2_exec_est_init(
    const params::depth2::controller_repository& param_repo,
    const tasking_map& map,
    rta::bi_tdgraph* graph) {
  auto* task_params = param_repo.parse_results<rta::task_alloc_params>();

  auto cache_starter = map.find("cache_starter")->second;
  auto cache_finisher = map.find("cache_finisher")->second;
  auto cache_transferer = map.find("cache_transferer")->second;
  auto cache_collector = map.find("cache_collector")->second;
  if (!task_params->exec_est.seed_enabled) {
    return;
  }
  /*
   * As part of seeding exec estimates, we set the last executed subtask for a
   * TAB. It's OK to declare/use here as local variables as this code only
   * runs during initialization.
   *
   * Collector, harvester not partitionable in depth 1 initialization, so they
   * have only been initialized as atomic tasks.
   */
  std::default_random_engine eng(
      std::chrono::system_clock::now().time_since_epoch().count());
  std::uniform_int_distribution<> dist(0, 1);
  if (0 == dist(eng)) {
    graph->tab_child(graph->root_tab(), graph->root_tab()->child1())
        ->last_subtask(cache_starter);
    graph->tab_child(graph->root_tab(), graph->root_tab()->child2())
        ->last_subtask(cache_transferer);
  } else {
    graph->tab_child(graph->root_tab(), graph->root_tab()->child1())
        ->last_subtask(cache_finisher);
    graph->tab_child(graph->root_tab(), graph->root_tab()->child2())
        ->last_subtask(cache_collector);
  }
  rmath::rangeu cs_bounds =
      task_params->exec_est.ranges.find("cache_starter")->second;
  rmath::rangeu cf_bounds =
      task_params->exec_est.ranges.find("cache_finisher")->second;
  rmath::rangeu ct_bounds =
      task_params->exec_est.ranges.find("cache_transferer")->second;
  rmath::rangeu cc_bounds =
      task_params->exec_est.ranges.find("cache_collector")->second;

  ER_INFO("Seeding exec estimate for tasks: '%s'=%s, '%s'=%s",
          cache_starter->name().c_str(),
          cs_bounds.to_str().c_str(),
          cache_finisher->name().c_str(),
          cf_bounds.to_str().c_str());
  cache_starter->exec_estimate_init(cs_bounds);
  cache_finisher->exec_estimate_init(cf_bounds);

  ER_INFO("Seeding exec estimate for tasks: '%s'=%s, '%s'=%s",
          cache_transferer->name().c_str(),
          ct_bounds.to_str().c_str(),
          cache_collector->name().c_str(),
          cc_bounds.to_str().c_str());
  cache_transferer->exec_estimate_init(ct_bounds);
  cache_collector->exec_estimate_init(cc_bounds);
} /* depth2_exec_est_init() */

std::unique_ptr<rta::bi_tdgraph_executive> tasking_initializer::operator()(
    const params::depth2::controller_repository& param_repo) {
  auto* task_params = param_repo.parse_results<rta::task_alloc_params>();
  auto graph = rcppsw::make_unique<rta::bi_tdgraph>(task_params);
  auto* execp = param_repo.parse_results<rta::task_executive_params>();

  auto map1 = depth1_tasks_create(param_repo, graph.get());
  depth1_exec_est_init(param_repo, map1, graph.get());

  auto map2 = depth2_tasks_create(param_repo, graph.get());
  depth2_exec_est_init(param_repo, map2, graph.get());

  graph->active_tab_init(execp->tab_init_method);
  return rcppsw::make_unique<rta::bi_tdgraph_executive>(execp, std::move(graph));
} /* initialize() */

NS_END(depth2, controller, fordyca);
