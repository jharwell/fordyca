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
#include "fordyca/controller/depth1/sensing_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/fsm/depth2/block_to_cache_site_fsm.hpp"
#include "fordyca/fsm/depth2/block_to_new_cache_fsm.hpp"
#include "fordyca/fsm/depth2/cache_transferer_fsm.hpp"
#include "fordyca/params/depth2/controller_repository.hpp"
#include "fordyca/params/depth2/exec_estimates_params.hpp"
#include "fordyca/tasks/depth1/collector.hpp"
#include "fordyca/tasks/depth2/cache_finisher.hpp"
#include "fordyca/tasks/depth2/cache_starter.hpp"
#include "fordyca/tasks/depth2/cache_transferer.hpp"
#include "fordyca/tasks/depth2/cache_collector.hpp"

#include "rcppsw/task_allocation/bi_tdgraph.hpp"
#include "rcppsw/task_allocation/bi_tdgraph_executive.hpp"
#include "rcppsw/task_allocation/task_allocation_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth2);
using ds::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
tasking_initializer::tasking_initializer(
    bool exec_ests_oracle,
    const controller::block_selection_matrix* bsel_matrix,
    const controller::cache_selection_matrix* csel_matrix,
    controller::saa_subsystem* const saa,
    base_perception_subsystem* const perception)
    : depth1::tasking_initializer(exec_ests_oracle,
                                  bsel_matrix,
                                  csel_matrix,
                                  saa,
                                  perception),
    ER_CLIENT_INIT("fordyca.controller.depth2.tasking_initializer") {}

tasking_initializer::~tasking_initializer(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void tasking_initializer::depth2_tasking_init(
    params::depth2::controller_repository* const param_repo) {
  auto* est_params =
      param_repo->parse_results<params::depth2::exec_estimates_params>();
  auto* task_params = param_repo->parse_results<ta::task_allocation_params>();

  std::unique_ptr<ta::taskable> cache_starter_fsm =
      rcppsw::make_unique<fsm::depth2::block_to_cache_site_fsm>(
          block_sel_matrix(),
          cache_sel_matrix(),
          saa_subsystem(),
          perception()->map());

  std::unique_ptr<ta::taskable> cache_finisher_fsm =
      rcppsw::make_unique<fsm::depth2::block_to_new_cache_fsm>(
          block_sel_matrix(),
          cache_sel_matrix(),
          saa_subsystem(),
          perception()->map());

  std::unique_ptr<ta::taskable> cache_transferer_fsm =
      rcppsw::make_unique<fsm::depth2::cache_transferer_fsm>(
          cache_sel_matrix(), saa_subsystem(), perception()->map());
  std::unique_ptr<ta::taskable> cache_collector_fsm =
      rcppsw::make_unique<fsm::depth1::cached_block_to_nest_fsm>(
          cache_sel_matrix(), saa_subsystem(), perception()->map());

  auto cache_starter =
      new tasks::depth2::cache_starter(task_params,
                                       std::move(cache_starter_fsm));
  auto cache_finisher =
      new tasks::depth2::cache_finisher(task_params,
                                        std::move(cache_finisher_fsm));
  auto cache_transferer =
      new tasks::depth2::cache_transferer(task_params,
                                          std::move(cache_transferer_fsm));
  auto cache_collector =
      new tasks::depth2::cache_collector(task_params,
                                         std::move(cache_collector_fsm));

  auto collector = graph()->find_vertex(
      tasks::depth1::foraging_task::kCollectorName);
  auto harvester = graph()->find_vertex(
      tasks::depth1::foraging_task::kHarvesterName);

  if (est_params->seed_enabled) {
    /*
     * Collector, harvester not partitionable in depth 1 initialization, so they
     * have only been initialized as atomic tasks.
     */
    if (0 == std::rand() % 2) {
      dynamic_cast<ta::partitionable_polled_task*>(collector)->last_partition(
          cache_transferer);
      dynamic_cast<ta::partitionable_polled_task*>(harvester)->last_partition(
          cache_starter);
    } else {
      dynamic_cast<ta::partitionable_polled_task*>(collector)->last_partition(
          cache_collector);
      dynamic_cast<ta::partitionable_polled_task*>(harvester)->last_partition(
          cache_finisher);
    }
    ER_INFO("Seeding exec estimate for tasks: '%s'=[%u,%u], '%s'=[%u,%u]",
            cache_starter->name().c_str(),
            est_params->cache_starter_range.GetMin(),
            est_params->cache_starter_range.GetMax(),
            cache_finisher->name().c_str(),
            est_params->cache_finisher_range.GetMin(),
            est_params->cache_finisher_range.GetMax());
    cache_starter->exec_est_bounds_init(est_params->cache_starter_range.GetMin(),
                                        est_params->cache_starter_range.GetMax());
    cache_finisher->exec_est_bounds_init(est_params->cache_finisher_range.GetMin(),
                                         est_params->cache_starter_range.GetMax());

    ER_INFO("Seeding exec estimate for tasks: '%s'=[%u,%u], '%s'=[%u,%u]",
            cache_transferer->name().c_str(),
            est_params->cache_transferer_range.GetMin(),
            est_params->cache_transferer_range.GetMax(),
            cache_collector->name().c_str(),
            est_params->cache_collector_range.GetMin(),
            est_params->cache_collector_range.GetMax());
    cache_transferer->exec_est_bounds_init(est_params->cache_transferer_range.GetMin(),
                                           est_params->cache_transferer_range.GetMax());
    cache_collector->exec_est_bounds_init(est_params->cache_collector_range.GetMin(),
                                          est_params->cache_collector_range.GetMax());
  }

  collector->set_partitionable(true);
  collector->set_atomic(false);
  harvester->set_partitionable(true);
  harvester->set_atomic(false);
  graph()->set_children(tasks::depth1::foraging_task::kHarvesterName,
                        std::vector<ta::polled_task*>(
                            {cache_starter, cache_finisher}));
  graph()->set_children(tasks::depth1::foraging_task::kCollectorName,
                        std::vector<ta::polled_task*>(
                            {cache_transferer, cache_collector}));
} /* depth2_tasking_init() */

std::unique_ptr<ta::bi_tdgraph_executive> tasking_initializer::operator()(
    params::depth2::controller_repository* const param_repo) {
  stateful_tasking_init(param_repo);
  depth1_tasking_init(param_repo);

  /* collector, forager tasks are now partitionable */
  auto children = graph()->children(graph()->root());
  for (auto& t : children) {
    t->set_partitionable(true);
  } /* for(&t..) */

  depth2_tasking_init(param_repo);

  return rcppsw::make_unique<ta::bi_tdgraph_executive>(
      exec_ests_oracle(), graph());
} /* initialize() */

NS_END(depth2, controller, fordyca);
