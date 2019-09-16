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

#include <boost/mpl/for_each.hpp>
#include <vector>

#include "rcppsw/metrics/tasks/bi_tab_metrics.hpp"
#include "rcppsw/metrics/tasks/bi_tab_metrics_collector.hpp"
#include "rcppsw/metrics/tasks/bi_tdgraph_metrics_collector.hpp"
#include "rcppsw/metrics/tasks/execution_metrics.hpp"
#include "rcppsw/metrics/tasks/execution_metrics_collector.hpp"
#include "rcppsw/mpl/typelist.hpp"
#include "rcppsw/ta/bi_tdgraph_executive.hpp"
#include "rcppsw/ta/ds/bi_tab.hpp"

#include "fordyca/config/metrics_config.hpp"
#include "fordyca/controller/depth1/bitd_mdpo_controller.hpp"
#include "fordyca/metrics/caches/lifecycle_metrics_collector.hpp"
#include "fordyca/metrics/caches/location_metrics.hpp"
#include "fordyca/metrics/caches/location_metrics_collector.hpp"
#include "fordyca/metrics/caches/utilization_metrics_collector.hpp"
#include "fordyca/metrics/collector_registerer.hpp"
#include "fordyca/repr/arena_cache.hpp"
#include "fordyca/support/base_cache_manager.hpp"
#include "fordyca/tasks/depth0/foraging_task.hpp"
#include "fordyca/tasks/depth1/foraging_task.hpp"

#include "cosm/fsm/metrics/collision_metrics.hpp"
#include "cosm/fsm/metrics/current_explore_locs_metrics_collector.hpp"
#include "cosm/fsm/metrics/current_vector_locs_metrics_collector.hpp"
#include "cosm/fsm/metrics/goal_acq_locs_metrics_collector.hpp"
#include "cosm/fsm/metrics/goal_acq_metrics.hpp"
#include "cosm/fsm/metrics/goal_acq_metrics_collector.hpp"
#include "cosm/fsm/metrics/movement_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1, detail);

using collector_typelist = rmpl::typelist<
    metrics::collector_registerer::type_wrap<cfmetrics::goal_acq_metrics_collector>,
    metrics::collector_registerer::type_wrap<
        cfmetrics::goal_acq_locs_metrics_collector>,
    metrics::collector_registerer::type_wrap<
        cfmetrics::current_explore_locs_metrics_collector>,
    metrics::collector_registerer::type_wrap<
        cfmetrics::current_vector_locs_metrics_collector>,
    metrics::collector_registerer::type_wrap<
        rmetrics::tasks::execution_metrics_collector>,
    metrics::collector_registerer::type_wrap<
        rmetrics::tasks::bi_tab_metrics_collector>,
    metrics::collector_registerer::type_wrap<
        rmetrics::tasks::bi_tdgraph_metrics_collector>,
    metrics::collector_registerer::type_wrap<
        metrics::caches::utilization_metrics_collector>,
    metrics::collector_registerer::type_wrap<
        metrics::caches::lifecycle_metrics_collector>,
    metrics::collector_registerer::type_wrap<
        metrics::caches::location_metrics_collector> >;
NS_END(detail);

using task0 = tasks::depth0::foraging_task;
using task1 = tasks::depth1::foraging_task;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
depth1_metrics_aggregator::depth1_metrics_aggregator(
    const config::metrics_config* const mconfig,
    const std::string& output_root)
    : depth0_metrics_aggregator(mconfig, output_root),
      ER_CLIENT_INIT("fordyca.support.depth1.metrics_aggregator") {
  metrics::collector_registerer::creatable_set creatable_set = {
      {typeid(cfmetrics::goal_acq_metrics_collector),
       "cache_acq_counts",
       "caches::acq_counts"},
      {typeid(cfmetrics::goal_acq_locs_metrics_collector),
       "cache_acq_locs",
       "caches::acq_locs"},
      {typeid(cfmetrics::current_explore_locs_metrics_collector),
       "cache_acq_explore_locs",
       "caches::acq_explore_locs"},
      {typeid(cfmetrics::current_vector_locs_metrics_collector),
       "cache_acq_vector_locs",
       "caches::acq_vector_locs"},
      {typeid(rmetrics::tasks::execution_metrics_collector),
       "task_execution_collector",
       "tasks::execution::" + std::string(task1::kCollectorName)},
      {typeid(rmetrics::tasks::execution_metrics_collector),
       "task_execution_harvester",
       "tasks::execution::" + std::string(task1::kHarvesterName)},
      {typeid(rmetrics::tasks::execution_metrics_collector),
       "task_execution_generalist",
       "tasks::execution::" + std::string(task0::kGeneralistName)},
      {typeid(rmetrics::tasks::bi_tab_metrics_collector),
       "task_tab_generalist",
       "tasks::tab::generalist"},
      {typeid(rmetrics::tasks::bi_tdgraph_metrics_collector),
       "task_distribution",
       "tasks::distribution"},
      {typeid(metrics::caches::utilization_metrics_collector),
       "cache_utilization",
       "caches::utilization"},
      {typeid(metrics::caches::lifecycle_metrics_collector),
       "cache_lifecycle",
       "caches::lifecycle"},
      {typeid(metrics::caches::location_metrics_collector),
       "cache_locations",
       "caches::locations"}};
  metrics::collector_registerer registerer(mconfig, creatable_set, this, 1);
  boost::mpl::for_each<detail::collector_typelist>(registerer);

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

void depth1_metrics_aggregator::task_start_cb(const rta::polled_task* const,
                                              const rta::ds::bi_tab* const tab) {
  /*
   * Depth [0,1,2] metrics aggregators are registered on the same executive,
   * so this function will be called for the task allocations for any depth,
   * so anything that is not in our TAB should be ignored.
   */
  if (!(tab->root()->name() == task0::kGeneralistName)) {
    return;
  }
  collect("tasks::tab::generalist", *tab);
} /* task_start_cb() */

NS_END(depth1, support, fordyca);
