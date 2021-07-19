/**
 * \file d2_metrics_manager.cpp
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
#include "fordyca/support/d2/d2_metrics_manager.hpp"

#include <boost/mpl/for_each.hpp>

#include "rcppsw/utils/maskable_enum.hpp"

#include "rcppsw/metrics/collector_registerer.hpp"
#include "cosm/ta/bi_tdgraph_executive.hpp"
#include "cosm/ta/ds/bi_tab.hpp"
#include "cosm/ta/metrics/bi_tab_metrics.hpp"
#include "cosm/ta/metrics/bi_tab_metrics_collector.hpp"
#include "cosm/ta/metrics/bi_tab_metrics_csv_sink.hpp"
#include "cosm/ta/metrics/bi_tdgraph_metrics_collector.hpp"
#include "cosm/ta/metrics/bi_tdgraph_metrics_csv_sink.hpp"
#include "cosm/ta/metrics/execution_metrics.hpp"
#include "cosm/ta/metrics/execution_metrics_collector.hpp"
#include "cosm/ta/metrics/execution_metrics_csv_sink.hpp"

#include "fordyca/controller/cognitive/d2/birtd_mdpo_controller.hpp"
#include "fordyca/metrics/caches/site_selection_metrics_collector.hpp"
#include "fordyca/metrics/caches/site_selection_metrics_csv_sink.hpp"
#include "fordyca/tasks/d0/foraging_task.hpp"
#include "fordyca/tasks/d1/foraging_task.hpp"
#include "fordyca/tasks/d2/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, d2);

using task0 = tasks::d0::foraging_task;
using task1 = tasks::d1::foraging_task;
using task2 = tasks::d2::foraging_task;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
d2_metrics_manager::d2_metrics_manager(
    const rmconfig::metrics_config* const mconfig,
    const cdconfig::grid2D_config* const gconfig,
    const fs::path& output_root,
    size_t n_block_clusters)
    : d1_metrics_manager(mconfig, gconfig, output_root, n_block_clusters),
      ER_CLIENT_INIT("fordyca.support.d2.metrics_manager") {
  register_standard(mconfig);

  /* Overwrite d1; we have a deeper decomposition now */
  collector_unregister("tasks::distribution");
  register_with_decomp_depth(mconfig, 2);

  /* setup metric collection for all collector groups in all sink groups */
  initialize();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void d2_metrics_manager::task_start_cb(
    RCPPSW_UNUSED const cta::polled_task* const task,
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

void d2_metrics_manager::task_finish_or_abort_cb(
    const cta::polled_task* const task) {
  collect("tasks::execution::" + task->name(),
          dynamic_cast<const ctametrics::execution_metrics&>(*task));
} /* task_finish_or_abort_cb() */

void d2_metrics_manager::register_standard(
    const rmconfig::metrics_config* const mconfig) {
  using sink_list = rmpl::typelist<
      rmpl::identity<ctametrics::bi_tab_metrics_csv_sink>,
      rmpl::identity<ctametrics::execution_metrics_csv_sink>,
      rmpl::identity<metrics::caches::site_selection_metrics_csv_sink>
    >;
  rmetrics::creatable_collector_set creatable_set = {
    { typeid(ctametrics::bi_tab_metrics_collector),
      "task_tab_harvester",
      "tasks::tab::harvester",
      rmetrics::output_mode::ekAPPEND },
    { typeid(ctametrics::bi_tab_metrics_collector),
      "task_tab_collector",
      "tasks::tab::collector",
      rmetrics::output_mode::ekAPPEND },
    { typeid(ctametrics::execution_metrics_collector),
      "task_execution_cache_starter",
      "tasks::execution::" + std::string(task2::kCacheStarterName),
      rmetrics::output_mode::ekAPPEND },
    { typeid(ctametrics::execution_metrics_collector),
      "task_execution_cache_finisher",
      "tasks::execution::" + std::string(task2::kCacheFinisherName),
      rmetrics::output_mode::ekAPPEND },
    { typeid(ctametrics::execution_metrics_collector),
      "task_execution_cache_transferer",
      "tasks::execution::" + std::string(task2::kCacheTransfererName),
      rmetrics::output_mode::ekAPPEND },
    { typeid(ctametrics::execution_metrics_collector),
      "task_execution_cache_collector",
      "tasks::execution::" + std::string(task2::kCacheCollectorName),
      rmetrics::output_mode::ekAPPEND },
    { typeid(metrics::caches::site_selection_metrics_collector),
      "cache_site_selection",
      "caches::site_selection",
      rmetrics::output_mode::ekAPPEND }
  };
  rmetrics::register_with_csv_sink csv(&mconfig->csv,
                                       creatable_set,
                                       this);
  rmetrics::collector_registerer<decltype(csv)> registerer(std::move(csv));
  boost::mpl::for_each<sink_list>(registerer);
} /* register_standard() */

NS_END(d2, support, fordyca);
