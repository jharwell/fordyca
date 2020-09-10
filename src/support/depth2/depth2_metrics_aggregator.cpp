/**
 * \file depth2_metrics_aggregator.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#include "rcppsw/utils/maskable_enum.hpp"

#include "cosm/metrics/collector_registerer.hpp"
#include "cosm/ta/bi_tdgraph_executive.hpp"
#include "cosm/ta/ds/bi_tab.hpp"
#include "cosm/ta/metrics/bi_tab_metrics.hpp"
#include "cosm/ta/metrics/bi_tab_metrics_collector.hpp"
#include "cosm/ta/metrics/bi_tdgraph_metrics_collector.hpp"
#include "cosm/ta/metrics/execution_metrics.hpp"
#include "cosm/ta/metrics/execution_metrics_collector.hpp"

#include "fordyca/controller/cognitive/depth2/birtd_mdpo_controller.hpp"
#include "fordyca/metrics/caches/site_selection_metrics_collector.hpp"
#include "fordyca/tasks/depth0/foraging_task.hpp"
#include "fordyca/tasks/depth1/foraging_task.hpp"
#include "fordyca/tasks/depth2/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);

using task0 = tasks::depth0::foraging_task;
using task1 = tasks::depth1::foraging_task;
using task2 = tasks::depth2::foraging_task;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
depth2_metrics_aggregator::depth2_metrics_aggregator(
    const cmconfig::metrics_config* const mconfig,
    const cdconfig::grid2D_config* const gconfig,
    const std::string& output_root)
    : depth1_metrics_aggregator(mconfig, gconfig, output_root),
      ER_CLIENT_INIT("fordyca.support.depth2.metrics_aggregator") {
  register_standard(mconfig);

  /* Overwrite depth1; we have a deeper decomposition now */
  collector_remove("tasks::distribution");
  register_with_decomp_depth(mconfig, 2);

  reset_all();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void depth2_metrics_aggregator::task_start_cb(
    RCSW_UNUSED const cta::polled_task* const task,
    const cta::ds::bi_tab* const tab) {
  /* Not using stochastic nbhd policy */
  if (nullptr == tab) {
    return;
  }
  if (task0::kGeneralistName == tab->root()->name()) {
    collect("tasks::tab::generalist", *tab);
  } else if (task1::kHarvesterName == tab->root()->name()) {
    collect("tasks::tab::harvester", *tab);
  } else if (task1::kCollectorName == tab->root()->name()) {
    collect("tasks::tab::collector", *tab);
  } else {
    ER_FATAL_SENTINEL("Bad task name '%s'", task->name().c_str());
  }
} /* task_start_cb() */

void depth2_metrics_aggregator::task_finish_or_abort_cb(
    const cta::polled_task* const task) {
  collect("tasks::execution::" + task->name(),
          dynamic_cast<const ctametrics::execution_metrics&>(*task));
} /* task_finish_or_abort_cb() */

void depth2_metrics_aggregator::register_standard(
    const cmconfig::metrics_config* const mconfig) {
  using collector_typelist = rmpl::typelist<
      rmpl::identity<ctametrics::bi_tab_metrics_collector>,
      rmpl::identity<ctametrics::execution_metrics_collector>,
      rmpl::identity<metrics::caches::site_selection_metrics_collector> >;
  cmetrics::collector_registerer<>::creatable_set creatable_set = {
      {typeid(ctametrics::bi_tab_metrics_collector),
       "task_tab_harvester",
       "tasks::tab::harvester",
       rmetrics::output_mode::ekAPPEND},
      {typeid(ctametrics::bi_tab_metrics_collector),
       "task_tab_collector",
       "tasks::tab::collector",
       rmetrics::output_mode::ekAPPEND},
      {typeid(ctametrics::execution_metrics_collector),
       "task_execution_cache_starter",
       "tasks::execution::" + std::string(task2::kCacheStarterName),
       rmetrics::output_mode::ekAPPEND},
      {typeid(ctametrics::execution_metrics_collector),
       "task_execution_cache_finisher",
       "tasks::execution::" + std::string(task2::kCacheFinisherName),
       rmetrics::output_mode::ekAPPEND},
      {typeid(ctametrics::execution_metrics_collector),
       "task_execution_cache_transferer",
       "tasks::execution::" + std::string(task2::kCacheTransfererName),
       rmetrics::output_mode::ekAPPEND},
      {typeid(ctametrics::execution_metrics_collector),
       "task_execution_cache_collector",
       "tasks::execution::" + std::string(task2::kCacheCollectorName),
       rmetrics::output_mode::ekAPPEND},
      {typeid(metrics::caches::site_selection_metrics_collector),
       "cache_site_selection",
       "caches::site_selection",
       rmetrics::output_mode::ekAPPEND}};

  cmetrics::collector_registerer<> registerer(mconfig, creatable_set, this);
  boost::mpl::for_each<collector_typelist>(registerer);
} /* register_standard() */

NS_END(depth2, support, fordyca);
