/**
 * \file base_fs_output_manager.cpp
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
#include "fordyca/argos/metrics/base_fs_output_manager.hpp"

#include <boost/mpl/for_each.hpp>

#include "rcppsw/mpl/typelist.hpp"
#include "rcppsw/utils/maskable_enum.hpp"
#include "rcppsw/metrics/file_sink_registerer.hpp"
#include "rcppsw/metrics/register_with_sink.hpp"
#include "rcppsw/metrics/register_using_config.hpp"
#include "rcppsw/mpl/identity.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"
#include "cosm/argos/convergence_calculator.hpp"
#include "cosm/metrics/specs.hpp"

#include "fordyca/controller/foraging_controller.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics_collector.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics_csv_sink.hpp"
#include "fordyca/metrics/tv/env_dynamics_metrics_collector.hpp"
#include "fordyca/metrics/tv/env_dynamics_metrics_csv_sink.hpp"
#include "fordyca/argos/support/argos_swarm_manager.hpp"
#include "fordyca/argos/support/tv/tv_manager.hpp"
#include "fordyca/metrics/specs.hpp"
#include "fordyca/argos/support/tv/fordyca_pd_adaptor.hpp"
#include "fordyca/argos/support/tv/env_dynamics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, metrics, detail);

using sink_list =
    rmpl::typelist<rmpl::identity<fmetrics::blocks::manipulation_metrics_csv_sink>,
                   rmpl::identity<fmetrics::tv::env_dynamics_metrics_csv_sink>>;

NS_END(detail);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_fs_output_manager::base_fs_output_manager(
    const rmconfig::metrics_config* const mconfig,
    const cdconfig::grid2D_config* const gconfig,
    const fs::path& output_root,
    size_t n_block_clusters)
    : ER_CLIENT_INIT("fordyca.argos.metrics.base_fs_output_manager"),
      fs_output_manager(mconfig, output_root) {
  /* register collectors from base class */
  auto dims2D = rmath::dvec2zvec(gconfig->dims, gconfig->resolution.v());
  register_with_arena_dims2D(mconfig, dims2D);
  register_with_n_block_clusters(mconfig, n_block_clusters);

  /* register collectors common to all of FORDYCA */
  rmetrics::creatable_collector_set creatable_set = {
    { typeid(fmetrics::blocks::manipulation_metrics_collector),
      fmspecs::blocks::kManipulation.xml,
      fmspecs::blocks::kManipulation.scoped,
      rmetrics::output_mode::ekAPPEND },
    { typeid(fmetrics::tv::env_dynamics_metrics_collector),
      cmspecs::tv::kEnvironment.xml,
      cmspecs::tv::kEnvironment.scoped,
      rmetrics::output_mode::ekAPPEND },
  };

      rmetrics::register_with_sink<base_fs_output_manager,
                                   rmetrics::file_sink_registerer> csv(this,
                                                                       creatable_set);
  rmetrics::register_using_config<decltype(csv),
                                  rmetrics::config::file_sink_config> registerer(
                                      std::move(csv),
                                      &mconfig->csv);
  boost::mpl::for_each<detail::sink_list>(registerer);

  /* setup metric collection for all collector groups in all sink groups */
  initialize();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_fs_output_manager::collect_from_sm(
    const fasupport::argos_swarm_manager* const sm) {
  if (nullptr != sm->conv_calculator()) {
    collect(cmspecs::kConvergence.scoped, sm->conv_calculator()->decoratee());
  }

  collect(cmspecs::tv::kEnvironment.scoped,
          *sm->tv_manager()->dynamics<ctv::dynamics_type::ekENVIRONMENT>());

  if (nullptr !=
      sm->tv_manager()->dynamics<ctv::dynamics_type::ekPOPULATION>()) {
    collect(cmspecs::tv::kPopulation.scoped,
            *sm->tv_manager()->dynamics<ctv::dynamics_type::ekPOPULATION>());
  }
  collect_from_arena(sm->arena_map());
} /* collect_from_sm() */

NS_END(metrics, argos, fordyca);
