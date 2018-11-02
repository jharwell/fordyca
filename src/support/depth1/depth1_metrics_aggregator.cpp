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
#include "fordyca/metrics/caches/utilization_metrics_collector.hpp"
#include "fordyca/metrics/fsm/collision_metrics.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics_collector.hpp"
#include "fordyca/metrics/fsm/movement_metrics.hpp"
#include "fordyca/params/metrics_params.hpp"
#include "rcppsw/metrics/tasks/bi_tab_metrics.hpp"
#include "rcppsw/metrics/tasks/bi_tab_metrics_collector.hpp"
#include "rcppsw/metrics/tasks/bi_tdgraph_metrics_collector.hpp"
#include "rcppsw/metrics/tasks/execution_metrics.hpp"
#include "rcppsw/metrics/tasks/execution_metrics_collector.hpp"
#include "rcppsw/task_allocation/bi_tab.hpp"
#include "rcppsw/task_allocation/bi_tdgraph_executive.hpp"

#include "fordyca/controller/depth1/greedy_partitioning_controller.hpp"
#include "fordyca/representation/arena_cache.hpp"
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
    const struct params::metrics_params* params,
    const std::string& output_root)
    : stateful_metrics_aggregator(params, output_root),
      ER_CLIENT_INIT("fordyca.support.depth1.metrics_aggregator") {
  register_collector<metrics::fsm::goal_acquisition_metrics_collector>(
      "caches::acquisition",
      metrics_path() + "/" + params->cache_acquisition_fname,
      params->collect_interval);

  register_collector<rcppsw::metrics::tasks::execution_metrics_collector>(
      "tasks::execution::" + std::string(task1::kCollectorName),
      metrics_path() + "/" + params->task_execution_collector_fname,
      params->collect_interval);
  register_collector<rcppsw::metrics::tasks::execution_metrics_collector>(
      "tasks::execution::" + std::string(task1::kHarvesterName),
      metrics_path() + "/" + params->task_execution_harvester_fname,
      params->collect_interval);
  register_collector<rcppsw::metrics::tasks::execution_metrics_collector>(
      "tasks::execution::" + std::string(task0::kGeneralistName),
      metrics_path() + "/" + params->task_execution_generalist_fname,
      params->collect_interval);

  register_collector<rcppsw::metrics::tasks::bi_tab_metrics_collector>(
      "tasks::tab::generalist",
      metrics_path() + "/" + params->task_generalist_tab_fname,
      params->collect_interval);

  register_collector<rcppsw::metrics::tasks::bi_tdgraph_metrics_collector>(
      "tasks::distribution",
      metrics_path() + "/" + params->task_distribution_fname,
      params->collect_interval,
      1);

  register_collector<metrics::caches::utilization_metrics_collector>(
      "caches::utilization",
      metrics_path() + "/" + params->cache_utilization_fname,
      params->collect_interval);
  register_collector<metrics::caches::lifecycle_metrics_collector>(
      "caches::lifecycle",
      metrics_path() + "/" + params->cache_lifecycle_fname,
      params->collect_interval);
  reset_all();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void depth1_metrics_aggregator::collect_from_cache(
    const representation::arena_cache* const cache) {
  collect("caches::utilization", *cache);
} /* collect_from_cache() */

void depth1_metrics_aggregator::collect_from_cache_manager(
    const support::base_cache_manager* const manager) {
  collect("caches::lifecycle", *manager);
} /* collect_from_cache() */

void depth1_metrics_aggregator::task_finish_or_abort_cb(
    const ta::polled_task* const task) {
  /*
   * Both depth1 and depth2 metrics aggregators are registered on the same
   * callback, so this function will be called for the depth2 task abort/finish
   * as well, which should be ignored.
   */
  if (!(task1::task_in_depth1(task) || task0::task_in_depth0(task))) {
      return;
  }
  collect("tasks::execution::" + task->name(),
          dynamic_cast<const rcppsw::metrics::tasks::execution_metrics&>(*task));
} /* task_finish_or_abort_cb() */


void depth1_metrics_aggregator::task_alloc_cb(
    const ta::polled_task* const,
    const ta::bi_tab* const tab) {
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
