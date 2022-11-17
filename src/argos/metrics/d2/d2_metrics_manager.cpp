/**
 * \file d2_metrics_manager.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/argos/metrics/d2/d2_metrics_manager.hpp"

#include <boost/mpl/for_each.hpp>

#include "rcppsw/metrics/file_sink_registerer.hpp"
#include "rcppsw/metrics/register_using_config.hpp"
#include "rcppsw/metrics/register_with_sink.hpp"
#include "rcppsw/mpl/identity.hpp"
#include "rcppsw/utils/maskable_enum.hpp"

#include "cosm/ds/cell2D.hpp"
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
NS_START(fordyca, argos, metrics, d2);

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
      ER_CLIENT_INIT("fordyca.argos.metrics.d2.metrics_manager") {
  register_standard(mconfig);

  /* Overwrite d1; we have a deeper decomposition now */
  collector_unregister(cmspecs::tasks::kDistribution.scoped());
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
    collect(fmspecs::tasks::tab::kGeneralist.scoped(), *tab);
  } else if (task1::kHarvesterName == tab->root()->name()) {
    collect(fmspecs::tasks::tab::kHarvester.scoped(), *tab);
  } else if (task1::kCollectorName == tab->root()->name()) {
    collect(fmspecs::tasks::tab::kCollector.scoped(), *tab);
  } else {
    ER_FATAL_SENTINEL("Bad task name '%s'", task->name().c_str());
  }
} /* task_start_cb() */

void d2_metrics_manager::task_finish_or_abort_cb(
    const cta::polled_task* const task) {
  collect("tasks/execution/" + task->name(),
          dynamic_cast<const ctametrics::execution_metrics&>(*task));
} /* task_finish_or_abort_cb() */

void d2_metrics_manager::register_standard(
    const rmconfig::metrics_config* const mconfig) {
  using sink_list = rmpl::typelist<
      rmpl::identity<ctametrics::bi_tab_metrics_csv_sink>,
      rmpl::identity<ctametrics::execution_metrics_csv_sink>,
      rmpl::identity<fmetrics::caches::site_selection_metrics_csv_sink> >;
  rmetrics::creatable_collector_set creatable_set = {
    { typeid(ctametrics::bi_tab_metrics_collector),
      fmspecs::tasks::tab::kHarvester.xml(),
      fmspecs::tasks::tab::kHarvester.scoped(),
      rmetrics::output_mode::ekAPPEND },
    { typeid(ctametrics::bi_tab_metrics_collector),
      fmspecs::tasks::tab::kCollector.xml(),
      fmspecs::tasks::tab::kCollector.scoped(),
      rmetrics::output_mode::ekAPPEND },
    { typeid(ctametrics::execution_metrics_collector),
      fmspecs::tasks::exec::kCacheStarter.xml(),
      fmspecs::tasks::exec::kCacheStarter.scoped(),
      rmetrics::output_mode::ekAPPEND },
    { typeid(ctametrics::execution_metrics_collector),
      fmspecs::tasks::exec::kCacheFinisher.xml(),
      fmspecs::tasks::exec::kCacheFinisher.scoped(),
      rmetrics::output_mode::ekAPPEND },
    { typeid(ctametrics::execution_metrics_collector),
      fmspecs::tasks::exec::kCacheTransferer.xml(),
      fmspecs::tasks::exec::kCacheTransferer.scoped(),
      rmetrics::output_mode::ekAPPEND },
    { typeid(ctametrics::execution_metrics_collector),
      fmspecs::tasks::exec::kCacheCollector.xml(),
      fmspecs::tasks::exec::kCacheCollector.scoped(),
      rmetrics::output_mode::ekAPPEND },
    { typeid(fmetrics::caches::site_selection_metrics_collector),
      fmspecs::caches::kSiteSelection.xml(),
      fmspecs::caches::kSiteSelection.scoped(),
      rmetrics::output_mode::ekAPPEND }
  };
  rmetrics::register_with_sink<d2_metrics_manager, rmetrics::file_sink_registerer>
      csv(this, creatable_set);
  rmetrics::register_using_config<decltype(csv),
                                  rmetrics::config::file_sink_config>
      registerer(std::move(csv), &mconfig->csv);
  boost::mpl::for_each<sink_list>(registerer);
} /* register_standard() */

NS_END(d2, metrics, argos, fordyca);
