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
#include <boost/mpl/for_each.hpp>

#include "fordyca/config/metrics_config.hpp"
#include "fordyca/controller/depth2/grp_mdpo_controller.hpp"
#include "fordyca/metrics/collector_registerer.hpp"
#include "fordyca/tasks/depth0/foraging_task.hpp"
#include "fordyca/tasks/depth1/foraging_task.hpp"
#include "fordyca/tasks/depth2/foraging_task.hpp"
#include "rcppsw/metrics/tasks/bi_tab_metrics.hpp"
#include "rcppsw/metrics/tasks/bi_tab_metrics_collector.hpp"
#include "rcppsw/metrics/tasks/bi_tdgraph_metrics_collector.hpp"
#include "rcppsw/metrics/tasks/execution_metrics.hpp"
#include "rcppsw/metrics/tasks/execution_metrics_collector.hpp"
#include "rcppsw/ta/bi_tab.hpp"
#include "rcppsw/ta/bi_tdgraph_executive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2, detail);
using collector_typelist =
    rmpl::typelist<metrics::collector_registerer::type_wrap<
                       rmetrics::tasks::bi_tab_metrics_collector>,
                   metrics::collector_registerer::type_wrap<
                     rmetrics::tasks::execution_metrics_collector>,
                   metrics::collector_registerer::type_wrap<
                     rmetrics::tasks::bi_tdgraph_metrics_collector> >;
NS_END(detail);

using acq_goal_type = metrics::fsm::goal_acq_metrics::goal_type;
using task0 = tasks::depth0::foraging_task;
using task1 = tasks::depth1::foraging_task;
using task2 = tasks::depth2::foraging_task;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
depth2_metrics_aggregator::depth2_metrics_aggregator(
    const config::metrics_config* const mconfig,
    const std::string& output_root)
    : depth1_metrics_aggregator(mconfig, output_root),
      ER_CLIENT_INIT("fordyca.support.depth2.metrics_aggregator") {
  metrics::collector_registerer::creatable_set creatable_set = {
      {typeid(rmetrics::tasks::bi_tab_metrics_collector),
       "task_tab_harvester",
       "tasks::tab::harvester"},
      {typeid(rmetrics::tasks::bi_tab_metrics_collector),
       "task_tab_collector",
       "tasks::tab::collector"},
      {typeid(rmetrics::tasks::execution_metrics_collector),
       "task_execution_cache_starter",
       "tasks::execution::" + std::string(task2::kCacheStarterName)},
      {typeid(rmetrics::tasks::execution_metrics_collector),
       "task_execution_cache_finisher",
       "tasks::execution::" + std::string(task2::kCacheFinisherName)},
      {typeid(rmetrics::tasks::execution_metrics_collector),
       "task_execution_cache_transferer",
       "tasks::execution::" + std::string(task2::kCacheTransfererName)},
      {typeid(rmetrics::tasks::execution_metrics_collector),
       "task_execution_cache_collector",
       "tasks::execution::" + std::string(task2::kCacheCollectorName)},
      {typeid(rmetrics::tasks::bi_tdgraph_metrics_collector),
       "task_distribution",
       "tasks::distribution"}};

  /* Overwrite depth1; we have a deeper decomposition now */
  collector_remove("tasks::distribution");

  metrics::collector_registerer registerer(mconfig, creatable_set, this, 2);
  boost::mpl::for_each<detail::collector_typelist>(registerer);

  reset_all();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void depth2_metrics_aggregator::task_alloc_cb(
    __rcsw_unused const rta::polled_task* const task,
    const rta::bi_tab* const tab) {
  if (task0::kGeneralistName == tab->root()->name()) {
    collect("tasks::tab::generalist", *tab);
  } else if (task1::kHarvesterName == tab->root()->name()) {
    collect("tasks::tab::harvester", *tab);
  } else if (task1::kCollectorName == tab->root()->name()) {
    collect("tasks::tab::collector", *tab);
  } else {
    ER_FATAL_SENTINEL("Bad task name '%s'", task->name().c_str());
  }
} /* task_alloc_cb() */

void depth2_metrics_aggregator::task_finish_or_abort_cb(
    const rta::polled_task* const task) {
  collect("tasks::execution::" + task->name(),
          dynamic_cast<const rmetrics::tasks::execution_metrics&>(*task));
} /* task_finish_or_abort_cb() */

NS_END(depth2, support, fordyca);
