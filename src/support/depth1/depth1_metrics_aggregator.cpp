/**
 * @file depth1_metrics_aggregator.cpp
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
#include "fordyca/support/depth1/depth1_metrics_aggregator.hpp"
#include <vector>

#include "fordyca/metrics/caches/lifecycle_metrics_collector.hpp"
#include "fordyca/metrics/caches/location_metrics.hpp"
#include "fordyca/metrics/caches/location_metrics_collector.hpp"
#include "fordyca/metrics/caches/utilization_metrics_collector.hpp"
#include "fordyca/metrics/fsm/acquisition_loc_metrics_collector.hpp"
#include "fordyca/metrics/fsm/collision_metrics.hpp"
#include "fordyca/metrics/fsm/current_explore_loc_metrics_collector.hpp"
#include "fordyca/metrics/fsm/current_vector_loc_metrics_collector.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics_collector.hpp"
#include "fordyca/metrics/fsm/movement_metrics.hpp"
#include "fordyca/params/metrics_params.hpp"

#include "rcppsw/metrics/tasks/bi_tab_metrics.hpp"
#include "rcppsw/metrics/tasks/bi_tab_metrics_collector.hpp"
#include "rcppsw/metrics/tasks/bi_tdgraph_metrics_collector.hpp"
#include "rcppsw/metrics/tasks/execution_metrics.hpp"
#include "rcppsw/metrics/tasks/execution_metrics_collector.hpp"
#include "rcppsw/ta/bi_tab.hpp"
#include "rcppsw/ta/bi_tdgraph_executive.hpp"

#include "fordyca/controller/depth1/gp_mdpo_controller.hpp"
#include "fordyca/repr/arena_cache.hpp"
#include "fordyca/support/base_cache_manager.hpp"
#include "fordyca/tasks/depth0/foraging_task.hpp"
#include "fordyca/tasks/depth1/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);
using task0 = tasks::depth0::foraging_task;
using task1 = tasks::depth1::foraging_task;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
depth1_metrics_aggregator::depth1_metrics_aggregator(
    const params::metrics_params* const mparams,
    const std::string& output_root)
    : depth0_metrics_aggregator(mparams, output_root),
      ER_CLIENT_INIT("fordyca.support.depth1.metrics_aggregator") {
  register_collector<metrics::fsm::goal_acquisition_metrics_collector>(
      "caches::acq_counts",
      metrics_path() + "/" + mparams->cache_acq_counts_fname,
      mparams->collect_interval);
  register_collector<metrics::fsm::acquisition_loc_metrics_collector>(
      "caches::acq_locs",
      metrics_path() + "/" + mparams->cache_acq_locs_fname,
      mparams->collect_interval,
      rmath::dvec2uvec(mparams->arena_grid.upper,
                       mparams->arena_grid.resolution));

  register_collector<metrics::fsm::current_explore_loc_metrics_collector>(
      "caches::acq_explore_locs",
      metrics_path() + "/" + mparams->cache_acq_explore_locs_fname,
      mparams->collect_interval,
      rmath::dvec2uvec(mparams->arena_grid.upper,
                       mparams->arena_grid.resolution));
  register_collector<metrics::fsm::current_vector_loc_metrics_collector>(
      "caches::acq_vector_locs",
      metrics_path() + "/" + mparams->cache_acq_vector_locs_fname,
      mparams->collect_interval,
      rmath::dvec2uvec(mparams->arena_grid.upper,
                       mparams->arena_grid.resolution));

  register_collector<rmetrics::tasks::execution_metrics_collector>(
      "tasks::execution::" + std::string(task1::kCollectorName),
      metrics_path() + "/" + mparams->task_execution_collector_fname,
      mparams->collect_interval);
  register_collector<rmetrics::tasks::execution_metrics_collector>(
      "tasks::execution::" + std::string(task1::kHarvesterName),
      metrics_path() + "/" + mparams->task_execution_harvester_fname,
      mparams->collect_interval);
  register_collector<rmetrics::tasks::execution_metrics_collector>(
      "tasks::execution::" + std::string(task0::kGeneralistName),
      metrics_path() + "/" + mparams->task_execution_generalist_fname,
      mparams->collect_interval);

  register_collector<rmetrics::tasks::bi_tab_metrics_collector>(
      "tasks::tab::generalist",
      metrics_path() + "/" + mparams->task_tab_generalist_fname,
      mparams->collect_interval);

  register_collector<rmetrics::tasks::bi_tdgraph_metrics_collector>(
      "tasks::distribution",
      metrics_path() + "/" + mparams->task_distribution_fname,
      mparams->collect_interval,
      1);

  register_collector<metrics::caches::utilization_metrics_collector>(
      "caches::utilization",
      metrics_path() + "/" + mparams->cache_utilization_fname,
      mparams->collect_interval);
  register_collector<metrics::caches::lifecycle_metrics_collector>(
      "caches::lifecycle",
      metrics_path() + "/" + mparams->cache_lifecycle_fname,
      mparams->collect_interval);

  register_collector<metrics::caches::location_metrics_collector>(
      "caches::locations",
      metrics_path() + "/" + mparams->cache_locations_fname,
      mparams->collect_interval,
      rmath::dvec2uvec(mparams->arena_grid.upper,
                       mparams->arena_grid.resolution));
  reset_all();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void depth1_metrics_aggregator::collect_from_cache(
    const repr::arena_cache* const cache) {
  auto util_m = dynamic_cast<const metrics::caches::utilization_metrics*>(cache);
  auto loc_m = dynamic_cast<const metrics::caches::location_metrics*>(cache);
  collect("caches::utilization", *util_m);
  collect("caches::locations", *loc_m);
} /* collect_from_cache() */

void depth1_metrics_aggregator::collect_from_cache_manager(
    const support::base_cache_manager* const manager) {
  collect("caches::lifecycle", *manager);
} /* collect_from_cache() */

void depth1_metrics_aggregator::task_finish_or_abort_cb(
    const rta::polled_task* const task) {
  /*
   * Both depth1 and depth2 metrics aggregators are registered on the same
   * callback, so this function will be called for the depth2 task abort/finish
   * as well, which should be ignored.
   */
  if (!(task1::task_in_depth1(task) || task0::task_in_depth0(task))) {
    return;
  }
  collect("tasks::execution::" + task->name(),
          dynamic_cast<const rmetrics::tasks::execution_metrics&>(*task));
} /* task_finish_or_abort_cb() */

void depth1_metrics_aggregator::task_alloc_cb(const rta::polled_task* const,
                                              const rta::bi_tab* const tab) {
  /*
   * Depth [0,1,2] metrics aggregators are registered on the same executive,
   * so this function will be called for the task allocations for any depth,
   * so anything that is not in our TAB should be ignored.
   */
  if (!(tab->root()->name() == task0::kGeneralistName)) {
    return;
  }
  collect("tasks::tab::generalist", *tab);
} /* task_alloc_cb() */

NS_END(depth1, support, fordyca);
