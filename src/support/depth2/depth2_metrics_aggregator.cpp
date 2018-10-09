/**
 * @file depth2_metrics_aggregator.cpp
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
#include "fordyca/support/depth2/depth2_metrics_aggregator.hpp"
#include <vector>

#include "fordyca/params/metrics_params.hpp"
#include "rcppsw/metrics/tasks/bi_tab_metrics.hpp"
#include "rcppsw/metrics/tasks/bi_tab_metrics_collector.hpp"
#include "rcppsw/metrics/tasks/execution_metrics.hpp"
#include "rcppsw/metrics/tasks/execution_metrics_collector.hpp"
#include "rcppsw/task_allocation/bi_tab.hpp"
#include "rcppsw/task_allocation/bi_tdgraph_executive.hpp"

#include "fordyca/controller/depth2/greedy_recpart_controller.hpp"
#include "fordyca/tasks/depth0/foraging_task.hpp"
#include "fordyca/tasks/depth1/foraging_task.hpp"
#include "fordyca/tasks/depth2/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);
using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using task0 = tasks::depth0::foraging_task;
using task1 = tasks::depth1::foraging_task;
using task2 = tasks::depth2::foraging_task;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
depth2_metrics_aggregator::depth2_metrics_aggregator(
    const struct params::metrics_params* params,
    const std::string& output_root)
    : depth1_metrics_aggregator(params, output_root),
      ER_CLIENT_INIT("fordyca.support.depth2.metrics_aggregator") {
  register_collector<rcppsw::metrics::tasks::bi_tab_metrics_collector>(
      "tasks::collector_tab",
      metrics_path() + "/" + params->task_collector_tab_fname,
      params->collect_interval);
  register_collector<rcppsw::metrics::tasks::bi_tab_metrics_collector>(
      "tasks::harvester_tab",
      metrics_path() + "/" + params->task_harvester_tab_fname,
      params->collect_interval);
  register_collector<rcppsw::metrics::tasks::execution_metrics_collector>(
      "tasks::execution::" + std::string(task2::kCacheStarterName),
      metrics_path() + "/" + params->task_execution_generalist_fname,
      params->collect_interval);
  register_collector<rcppsw::metrics::tasks::execution_metrics_collector>(
      "tasks::execution::" + std::string(task2::kCacheFinisherName),
      metrics_path() + "/" + params->task_execution_collector_fname,
      params->collect_interval);
  register_collector<rcppsw::metrics::tasks::execution_metrics_collector>(
      "tasks::execution::" + std::string(task2::kCacheTransfererName),
      metrics_path() + "/" + params->task_execution_harvester_fname,
      params->collect_interval);
  register_collector<rcppsw::metrics::tasks::execution_metrics_collector>(
      "tasks::execution::" + std::string(task2::kCacheCollectorName),
      metrics_path() + "/" + params->task_execution_harvester_fname,
      params->collect_interval);

  reset_all();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void depth2_metrics_aggregator::task_alloc_cb(
    const ta::polled_task* const task,
    const ta::bi_tab* const tab) {
  if (task0::kGeneralistName == task->name()) {
    collect("tasks::generalist_tab", *tab);
  } else if (task1::kHarvesterName == task->name()) {
    collect("tasks::harvester_tab", *tab);
  } else if (task1::kCollectorName == task->name()) {
    collect("tasks::collector_tab", *tab);
  } else {
    ER_FATAL_SENTINEL("Bad task name %s", task->name().c_str());
  }
} /* task_alloc_cb() */

NS_END(depth2, support, fordyca);
