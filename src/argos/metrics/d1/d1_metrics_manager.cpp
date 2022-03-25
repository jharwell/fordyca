/**
 * \file d1_metrics_manager.cpp
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
#include "fordyca/argos/metrics/d1/d1_metrics_manager.hpp"

#include <boost/mpl/for_each.hpp>
#include <vector>

#include "rcppsw/mpl/typelist.hpp"
#include "rcppsw/utils/maskable_enum.hpp"
#include "rcppsw/metrics/register_with_sink.hpp"
#include "rcppsw/metrics/register_using_config.hpp"
#include "rcppsw/metrics/file_sink_registerer.hpp"
#include "rcppsw/mpl/identity.hpp"

#include "cosm/arena/metrics/caches/location_metrics.hpp"
#include "cosm/arena/metrics/caches/location_metrics_collector.hpp"
#include "cosm/arena/metrics/caches/location_metrics_csv_sink.hpp"
#include "cosm/arena/metrics/caches/utilization_metrics.hpp"
#include "cosm/arena/metrics/caches/utilization_metrics_collector.hpp"
#include "cosm/arena/metrics/caches/utilization_metrics_csv_sink.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/spatial/metrics/explore_locs2D_metrics_collector.hpp"
#include "cosm/spatial/metrics/explore_locs2D_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/goal_acq_locs2D_metrics_collector.hpp"
#include "cosm/spatial/metrics/goal_acq_locs2D_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/goal_acq_metrics.hpp"
#include "cosm/spatial/metrics/goal_acq_metrics_collector.hpp"
#include "cosm/spatial/metrics/goal_acq_metrics_csv_sink.hpp"
#include "cosm/spatial/metrics/vector_locs2D_metrics_collector.hpp"
#include "cosm/spatial/metrics/vector_locs2D_metrics_csv_sink.hpp"
#include "cosm/ta/bi_tdgraph_executive.hpp"
#include "cosm/ta/ds/bi_tab.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/ta/metrics/bi_tab_metrics.hpp"
#include "cosm/ta/metrics/bi_tab_metrics_collector.hpp"
#include "cosm/ta/metrics/bi_tab_metrics_csv_sink.hpp"
#include "cosm/ta/metrics/bi_tdgraph_metrics_collector.hpp"
#include "cosm/ta/metrics/bi_tdgraph_metrics_csv_sink.hpp"
#include "cosm/ta/metrics/execution_metrics.hpp"
#include "cosm/ta/metrics/execution_metrics_collector.hpp"
#include "cosm/ta/metrics/execution_metrics_csv_sink.hpp"

#include "fordyca/controller/cognitive/d1/bitd_mdpo_controller.hpp"
#include "fordyca/metrics/caches/lifecycle_metrics_collector.hpp"
#include "fordyca/metrics/caches/lifecycle_metrics_csv_sink.hpp"
#include "fordyca/argos/support/caches/base_manager.hpp"
#include "fordyca/tasks/d0/foraging_task.hpp"
#include "fordyca/tasks/d1/foraging_task.hpp"
#include "fordyca/metrics/specs.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, metrics, d1);

using task0 = tasks::d0::foraging_task;
using task1 = tasks::d1::foraging_task;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
d1_metrics_manager::d1_metrics_manager(
    const rmconfig::metrics_config* const mconfig,
    const cdconfig::grid2D_config* const gconfig,
    const fs::path& output_root,
    size_t n_block_clusters)
    : d0_metrics_manager(mconfig, gconfig, output_root, n_block_clusters),
      ER_CLIENT_INIT("fordyca.argos.metrics.d1.metrics_manager") {
  auto dims2D = rmath::dvec2zvec(gconfig->dims, gconfig->resolution.v());

  register_standard(mconfig);
  register_with_decomp_depth(mconfig, 1);
  register_with_arena_dims2D(mconfig, dims2D);

  /* setup metric collection for all collector groups in all sink groups */
  initialize();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void d1_metrics_manager::collect_from_cache(
    const carepr::arena_cache* const cache) {
  const auto* util_m =
      dynamic_cast<const cametrics::caches::utilization_metrics*>(cache);
  const auto* loc_m =
      dynamic_cast<const cametrics::caches::location_metrics*>(cache);
  collect(fmspecs::caches::kUtilization.scoped(), *util_m);
  collect(fmspecs::caches::kLocations.scoped(), *loc_m);
} /* collect_from_cache() */

void d1_metrics_manager::collect_from_cache_manager(
    const fascaches::base_manager* const manager) {
  collect(fmspecs::caches::kLifecycle.scoped(), *manager);
} /* collect_from_cache() */

void d1_metrics_manager::task_finish_or_abort_cb(
    const cta::polled_task* const task) {
  /*
   * Both d1 and d2 metrics managers are registered on the same
   * callback, so this function will be called for the d2 task abort/finish
   * as well, which should be ignored.
   */
  if (!(task1::task_in_d1(task) || task0::task_in_d0(task))) {
    return;
  }
  collect("tasks/execution/" + task->name(),
          dynamic_cast<const ctametrics::execution_metrics&>(*task));
} /* task_finish_or_abort_cb() */

void d1_metrics_manager::task_start_cb(const cta::polled_task* const,
                                       const cta::ds::bi_tab* const tab) {
  /* Not using stochastic nbhd policy */
  if (nullptr == tab) {
    return;
  }
  /*
   * Depth [0,1,2] metrics managers are registered on the same executive,
   * so this function will be called for the task allocations for any depth,
   * so anything that is not in our TAB should be ignored.
   */
  if (!(tab->root()->name() == task0::kGeneralistName)) {
    return;
  }
  collect(fmspecs::tasks::tab::kGeneralist.scoped(), *tab);
} /* task_start_cb() */

void d1_metrics_manager::register_standard(
    const rmconfig::metrics_config* mconfig) {
  using sink_list = rmpl::typelist<
      rmpl::identity<csmetrics::goal_acq_metrics_csv_sink>,
      rmpl::identity<ctametrics::execution_metrics_csv_sink>,
      rmpl::identity<ctametrics::bi_tab_metrics_csv_sink>,
      rmpl::identity<cametrics::caches::utilization_metrics_csv_sink>,
      rmpl::identity<fmetrics::caches::lifecycle_metrics_csv_sink> >;
  rmetrics::creatable_collector_set creatable_set = {
    {
      typeid(csmetrics::goal_acq_metrics_collector),
      fmspecs::caches::kAcqCounts.xml(),
      fmspecs::caches::kAcqCounts.scoped(),
      rmetrics::output_mode::ekAPPEND
    },
    {
      typeid(ctametrics::execution_metrics_collector),
      fmspecs::tasks::exec::kCollector.xml(),
      fmspecs::tasks::exec::kCollector.scoped(),
      rmetrics::output_mode::ekAPPEND
    },
    {
      typeid(ctametrics::execution_metrics_collector),
      fmspecs::tasks::exec::kHarvester.xml(),
      fmspecs::tasks::exec::kHarvester.scoped(),
      rmetrics::output_mode::ekAPPEND
    },
    {
      typeid(ctametrics::execution_metrics_collector),
      fmspecs::tasks::exec::kGeneralist.xml(),
      fmspecs::tasks::exec::kGeneralist.scoped(),
      rmetrics::output_mode::ekAPPEND
    },
    {
      typeid(ctametrics::bi_tab_metrics_collector),
      fmspecs::tasks::tab::kGeneralist.xml(),
      fmspecs::tasks::tab::kGeneralist.scoped(),
      rmetrics::output_mode::ekAPPEND
    },
    {
      typeid(cametrics::caches::utilization_metrics_collector),
      fmspecs::caches::kUtilization.xml(),
      fmspecs::caches::kUtilization.scoped(),
      rmetrics::output_mode::ekAPPEND
    },
    {
      typeid(fmetrics::caches::lifecycle_metrics_collector),
      fmspecs::caches::kLifecycle.xml(),
      fmspecs::caches::kLifecycle.scoped(),
      rmetrics::output_mode::ekAPPEND
    }
  };

    rmetrics::register_with_sink<d1_metrics_manager,
                              rmetrics::file_sink_registerer> csv(this,
                                                                  creatable_set);
  rmetrics::register_using_config<decltype(csv),
                                  rmetrics::config::file_sink_config> registerer(
                                      std::move(csv),
                                      &mconfig->csv);
  boost::mpl::for_each<sink_list>(registerer);
} /* register_standard() */

void d1_metrics_manager::register_with_decomp_depth(
    const rmconfig::metrics_config* const mconfig,
    size_t depth) {
  using sink_list =
      rmpl::typelist<rmpl::identity<ctametrics::bi_tdgraph_metrics_csv_sink> >;
  rmetrics::creatable_collector_set creatable_set = {
    {
      typeid(ctametrics::bi_tdgraph_metrics_collector),
      cmspecs::tasks::kDistribution.xml(),
      cmspecs::tasks::kDistribution.scoped(),
      rmetrics::output_mode::ekAPPEND
    }
  };
  auto extra_args = std::make_tuple(depth);

  rmetrics::register_with_sink<d1_metrics_manager,
                               rmetrics::file_sink_registerer,
                               decltype(extra_args)> csv(this,
                                                         creatable_set,
                                                         extra_args);
  rmetrics::register_using_config<decltype(csv),
                                  rmetrics::config::file_sink_config> registerer(
                                      std::move(csv),
                                      &mconfig->csv);
  boost::mpl::for_each<sink_list>(registerer);
} /* register_with_decomp_depth() */

void d1_metrics_manager::register_with_arena_dims2D(
    const rmconfig::metrics_config* const mconfig,
    const rmath::vector2z& dims) {
  using sink_list = rmpl::typelist<
      rmpl::identity<csmetrics::goal_acq_locs2D_metrics_csv_sink>,
      rmpl::identity<csmetrics::explore_locs2D_metrics_csv_sink>,
      rmpl::identity<csmetrics::vector_locs2D_metrics_csv_sink>,
      rmpl::identity<cametrics::caches::location_metrics_csv_sink> >;

  rmetrics::creatable_collector_set creatable_set = {
    {
      typeid(csmetrics::goal_acq_locs2D_metrics_collector),
      fmspecs::caches::kAcqLocs2D.xml(),
      fmspecs::caches::kAcqLocs2D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE
    },
    {
      typeid(csmetrics::explore_locs2D_metrics_collector),
      fmspecs::caches::kAcqExploreLocs2D.xml(),
      fmspecs::caches::kAcqExploreLocs2D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE
    },
    {
      typeid(csmetrics::vector_locs2D_metrics_collector),
      fmspecs::caches::kAcqVectorLocs2D.xml(),
      fmspecs::caches::kAcqVectorLocs2D.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE
    },
    {
      typeid(cametrics::caches::location_metrics_collector),
      fmspecs::caches::kLocations.xml(),
      fmspecs::caches::kLocations.scoped(),
      rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE
    }
  };

  auto extra_args = std::make_tuple(dims);

  rmetrics::register_with_sink<d1_metrics_manager,
                               rmetrics::file_sink_registerer,
                               decltype(extra_args)> csv(this,
                                                         creatable_set,
                                                         extra_args);
  rmetrics::register_using_config<decltype(csv),
                                  rmetrics::config::file_sink_config> registerer(
                                      std::move(csv),
                                      &mconfig->csv);
  boost::mpl::for_each<sink_list>(registerer);
} /* register_with_arena_dims2D() */

NS_END(d1, metrics, argos, fordyca);
