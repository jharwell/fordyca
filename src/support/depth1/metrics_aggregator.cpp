/**
 * @file metrics_aggregator.cpp
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
#include "fordyca/support/depth1/metrics_aggregator.hpp"
#include <vector>

#include "fordyca/metrics/caches/lifecycle_metrics_collector.hpp"
#include "fordyca/metrics/caches/utilization_metrics_collector.hpp"
#include "fordyca/metrics/fsm/distance_metrics.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics_collector.hpp"
#include "fordyca/params/metrics_params.hpp"
#include "rcppsw/metrics/tasks/bifurcating_tab_metrics.hpp"
#include "rcppsw/metrics/tasks/bifurcating_tab_metrics_collector.hpp"
#include "rcppsw/metrics/tasks/execution_metrics.hpp"
#include "rcppsw/metrics/tasks/execution_metrics_collector.hpp"
#include "rcppsw/task_allocation/bifurcating_tab.hpp"

#include "fordyca/controller/depth1/foraging_controller.hpp"
#include "fordyca/metrics/caches/lifecycle_collator.hpp"
#include "fordyca/representation/arena_cache.hpp"
#include "fordyca/tasks/depth0/foraging_task.hpp"
#include "fordyca/tasks/depth1/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);
using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using task0 = tasks::depth0::foraging_task;
using task1 = tasks::depth1::foraging_task;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
metrics_aggregator::metrics_aggregator(
    std::shared_ptr<rcppsw::er::server> server,
    const struct params::metrics_params* params,
    const std::string& output_root)
    : stateful_metrics_aggregator(server, params, output_root) {
  register_collector<metrics::fsm::goal_acquisition_metrics_collector>(
      "caches::acquisition",
      metrics_path() + "/" + params->cache_acquisition_fname,
      params->collect_interval);

  register_collector<rcppsw::metrics::tasks::execution_metrics_collector>(
      "tasks::execution::" + std::string(task0::kGeneralistName),
      metrics_path() + "/" + params->task_execution_generalist_fname,
      params->collect_interval);
  register_collector<rcppsw::metrics::tasks::execution_metrics_collector>(
      "tasks::execution::" + std::string(task1::kCollectorName),
      metrics_path() + "/" + params->task_execution_collector_fname,
      params->collect_interval);
  register_collector<rcppsw::metrics::tasks::execution_metrics_collector>(
      "tasks::execution::" + std::string(task1::kHarvesterName),
      metrics_path() + "/" + params->task_execution_harvester_fname,
      params->collect_interval);

  register_collector<rcppsw::metrics::tasks::bifurcating_tab_metrics_collector>(
      "tasks::generalist_tab",
      metrics_path() + "/" + params->task_generalist_tab_fname,
      params->collect_interval);

  register_collector<metrics::caches::utilization_metrics_collector>(
      "caches::utilization",
      metrics_path() + "/" + params->cache_utilization_fname,
      params->collect_interval);
  register_collector<metrics::caches::lifecycle_metrics_collector>(
      "caches::lifecycle",
      metrics_path() + "/" + params->cache_lifecycle_fname,
      params->collect_interval);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void metrics_aggregator::collect_from_controller(
    const controller::depth1::foraging_controller* const controller) {
  auto distance_m =
      dynamic_cast<const metrics::fsm::distance_metrics*>(controller);
  ER_ASSERT(distance_m,
            "FATAL: Controller does not provide FSM distance metrics");
  collect("fsm::distance", *distance_m);

  auto worldm_m = dynamic_cast<const metrics::world_model_metrics*>(controller);
  ER_ASSERT(worldm_m, "FATAL: Controller does not provide world model metrics");
  collect("perception::world_model", *worldm_m);

  if (nullptr != controller->current_task()) {
    collect("tasks::generalist_tab", *(controller->active_tab()));
    collect_if(
        "blocks::acquisition",
        *dynamic_cast<const metrics::fsm::goal_acquisition_metrics*>(
            controller->current_task()),
        [&](const rcppsw::metrics::base_metrics& metrics) {
          return acquisition_goal_type::kBlock ==
                 dynamic_cast<const metrics::fsm::goal_acquisition_metrics&>(
                     metrics)
                     .acquisition_goal();
        });
    collect_if(
        "caches::acquisition",
        *dynamic_cast<const metrics::fsm::goal_acquisition_metrics*>(
            controller->current_task()),
        [&](const rcppsw::metrics::base_metrics& metrics) {
          return acquisition_goal_type::kExistingCache ==
                 dynamic_cast<const metrics::fsm::goal_acquisition_metrics&>(
                     metrics)
                     .acquisition_goal();
        });

    collect("tasks::execution::" + dynamic_cast<const ta::logical_task*>(
                                       controller->current_task())
                                       ->name(),
            dynamic_cast<const rcppsw::metrics::tasks::execution_metrics&>(
                *controller->current_task()));
  }
} /* collect_from_controller() */

void metrics_aggregator::collect_from_cache(
    const representation::arena_cache* const cache) {
  collect("caches::utilization", *cache);
} /* collect_from_cache() */

void metrics_aggregator::collect_from_cache_collator(
    const metrics::caches::lifecycle_collator* const collator) {
  collect("caches::lifecycle", *collator);
} /* collect_from_cache() */

NS_END(depth1, support, fordyca);
