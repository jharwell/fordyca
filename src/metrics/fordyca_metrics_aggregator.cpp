/**
 * \file fordyca_metrics_aggregator.cpp
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
#include "fordyca/metrics/fordyca_metrics_aggregator.hpp"

#include <boost/mpl/for_each.hpp>

#include "rcppsw/mpl/typelist.hpp"

#include "cosm/metrics/collector_registerer.hpp"
#include "cosm/pal/argos_convergence_calculator.hpp"

#include "fordyca/controller/foraging_controller.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics_collector.hpp"
#include "fordyca/metrics/tv/env_dynamics_metrics_collector.hpp"
#include "fordyca/support/base_loop_functions.hpp"
#include "fordyca/support/tv/tv_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, detail);

using collector_typelist =
    rmpl::typelist<rmpl::identity<blocks::manipulation_metrics_collector>,
                   rmpl::identity<tv::env_dynamics_metrics_collector>>;

NS_END(detail);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
fordyca_metrics_aggregator::fordyca_metrics_aggregator(
    const cmconfig::metrics_config* const mconfig,
    const cdconfig::grid2D_config* const gconfig,
    const std::string& output_root)
    : ER_CLIENT_INIT("fordyca.metrics.aggregator"),
      base_metrics_aggregator(mconfig, output_root) {
  /* register collectors from base class */
  auto dims2D = rmath::dvec2zvec(gconfig->dims, gconfig->resolution.v());
  register_with_arena_dims2D(mconfig, dims2D);

  /* register collectors common to all of FORDYCA */
  cmetrics::collector_registerer<>::creatable_set creatable_set = {
      {typeid(blocks::manipulation_metrics_collector),
       "block_manipulation",
       "blocks::manipulation",
       rmetrics::output_mode::ekAPPEND},
      {typeid(tv::env_dynamics_metrics_collector),
       "tv_environment",
       "tv::environment",
       rmetrics::output_mode::ekAPPEND},
  };

  cmetrics::collector_registerer<> registerer(mconfig, creatable_set, this);
  boost::mpl::for_each<detail::collector_typelist>(registerer);
  reset_all();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void fordyca_metrics_aggregator::collect_from_loop(
    const support::base_loop_functions* const loop) {
  if (nullptr != loop->conv_calculator()) {
    collect("swarm::convergence", loop->conv_calculator()->decoratee());
  }

  collect("tv::environment",
          *loop->tv_manager()->dynamics<ctv::dynamics_type::ekENVIRONMENT>());

  if (nullptr !=
      loop->tv_manager()->dynamics<ctv::dynamics_type::ekPOPULATION>()) {
    collect("tv::population",
            *loop->tv_manager()->dynamics<ctv::dynamics_type::ekPOPULATION>());
  }
} /* collect_from_loop() */

NS_END(metrics, fordyca);
