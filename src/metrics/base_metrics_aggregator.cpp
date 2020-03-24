/**
 * \file base_metrics_aggregator.cpp
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
#include "fordyca/metrics/base_metrics_aggregator.hpp"

#include <boost/mpl/for_each.hpp>

#include "rcppsw/mpl/typelist.hpp"

#include "cosm/convergence/convergence_calculator.hpp"
#include "cosm/convergence/metrics/convergence_metrics.hpp"
#include "cosm/convergence/metrics/convergence_metrics_collector.hpp"
#include "cosm/fsm/metrics/collision_locs_metrics_collector.hpp"
#include "cosm/fsm/metrics/collision_metrics.hpp"
#include "cosm/fsm/metrics/collision_metrics_collector.hpp"
#include "cosm/fsm/metrics/current_explore_locs_metrics_collector.hpp"
#include "cosm/fsm/metrics/current_vector_locs_metrics_collector.hpp"
#include "cosm/fsm/metrics/goal_acq_locs_metrics_collector.hpp"
#include "cosm/fsm/metrics/goal_acq_metrics.hpp"
#include "cosm/fsm/metrics/goal_acq_metrics_collector.hpp"
#include "cosm/fsm/metrics/movement_metrics.hpp"
#include "cosm/fsm/metrics/movement_metrics_collector.hpp"
#include "cosm/metrics/blocks/transport_metrics_collector.hpp"
#include "cosm/metrics/spatial/dist2D_metrics.hpp"
#include "cosm/metrics/spatial/dist2D_pos_metrics_collector.hpp"
#include "cosm/repr/base_block2D.hpp"
#include "cosm/tv/metrics/population_dynamics_metrics.hpp"
#include "cosm/tv/metrics/population_dynamics_metrics_collector.hpp"

#include "fordyca/controller/foraging_controller.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics_collector.hpp"
#include "fordyca/metrics/collector_registerer.hpp"
#include "fordyca/metrics/tv/env_dynamics_metrics.hpp"
#include "fordyca/metrics/tv/env_dynamics_metrics_collector.hpp"
#include "fordyca/support/base_loop_functions.hpp"
#include "fordyca/support/tv/tv_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, detail);

using collector_typelist = rmpl::typelist<
    collector_registerer::type_wrap<cfsm::metrics::movement_metrics_collector>,
    collector_registerer::type_wrap<cfsm::metrics::collision_metrics_collector>,
    collector_registerer::type_wrap<cfsm::metrics::collision_locs_metrics_collector>,
    collector_registerer::type_wrap<cfsm::metrics::goal_acq_metrics_collector>,
    collector_registerer::type_wrap<cfsm::metrics::goal_acq_locs_metrics_collector>,
    collector_registerer::type_wrap<
        cfsm::metrics::current_explore_locs_metrics_collector>,
    collector_registerer::type_wrap<
        cfsm::metrics::current_vector_locs_metrics_collector>,
    collector_registerer::type_wrap<cmetrics::blocks::transport_metrics_collector>,
    collector_registerer::type_wrap<blocks::manipulation_metrics_collector>,
    collector_registerer::type_wrap<cmetrics::spatial::dist2D_pos_metrics_collector>,
    collector_registerer::type_wrap<
        cconvergence::metrics::convergence_metrics_collector>,
    collector_registerer::type_wrap<tv::env_dynamics_metrics_collector>,
    collector_registerer::type_wrap<
        ctvmetrics::population_dynamics_metrics_collector> >;

NS_END(detail);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_metrics_aggregator::base_metrics_aggregator(
    const cmconfig::metrics_config* const mconfig,
    const cdconfig::grid_config* const gconfig,
    const std::string& output_root)
    : ER_CLIENT_INIT("fordyca.metrics.base_aggregator"),
      m_metrics_path(fs::path(output_root) / mconfig->output_dir) {
  if (!fs::exists(m_metrics_path)) {
    fs::create_directories(m_metrics_path);
  } else {
    ER_WARN("Output metrics path '%s' already exists", m_metrics_path.c_str());
  }
  collector_registerer::creatable_set creatable_set = {
      {typeid(cfsm::metrics::movement_metrics_collector),
       "fsm_movement",
       "fsm::movement",
       rmetrics::output_mode::ekAPPEND},
      {typeid(cfsm::metrics::collision_locs_metrics_collector),
       "fsm_collision_locs",
       "fsm::collision_locs",
       rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE},
      {typeid(cfsm::metrics::collision_metrics_collector),
       "fsm_collision_counts",
       "fsm::collision_counts",
       rmetrics::output_mode::ekAPPEND},
      {typeid(cfsm::metrics::goal_acq_metrics_collector),
       "block_acq_counts",
       "blocks::acq_counts",
       rmetrics::output_mode::ekAPPEND},
      {typeid(cfsm::metrics::goal_acq_locs_metrics_collector),
       "block_acq_locs",
       "blocks::acq_locs",
       rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE},
      {typeid(cfsm::metrics::current_explore_locs_metrics_collector),
       "block_acq_explore_locs",
       "blocks::acq_explore_locs",
       rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE},
      {typeid(cfsm::metrics::current_vector_locs_metrics_collector),
       "block_acq_vector_locs",
       "blocks::acq_vector_locs",
       rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE},
      {typeid(cmetrics::blocks::transport_metrics_collector),
       "block_transport",
       "blocks::transport",
       rmetrics::output_mode::ekAPPEND},
      {typeid(blocks::manipulation_metrics_collector),
       "block_manipulation",
       "blocks::manipulation",
       rmetrics::output_mode::ekAPPEND},
      {typeid(cmetrics::spatial::dist2D_pos_metrics_collector),
       "swarm_dist2D_pos",
       "swarm::spatial_dist2D::pos",
       rmetrics::output_mode::ekTRUNCATE | rmetrics::output_mode::ekCREATE},
      {typeid(cconvergence::metrics::convergence_metrics_collector),
       "swarm_convergence",
       "swarm::convergence",
       rmetrics::output_mode::ekAPPEND},
      {typeid(tv::env_dynamics_metrics_collector),
       "tv_environment",
       "tv::environment",
       rmetrics::output_mode::ekAPPEND},
      {typeid(ctvmetrics::population_dynamics_metrics_collector),
       "tv_population",
       "tv::population",
       rmetrics::output_mode::ekAPPEND}};

  collector_registerer registerer(mconfig, gconfig, creatable_set, this);
  boost::mpl::for_each<detail::collector_typelist>(registerer);
  reset_all();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_metrics_aggregator::collect_from_loop(
    const support::base_loop_functions* const loop) {
  if (nullptr != loop->conv_calculator()) {
    collect("swarm::convergence", *loop->conv_calculator());
  }

  collect("tv::environment",
          *loop->tv_manager()->dynamics<ctv::dynamics_type::ekENVIRONMENT>());

  if (loop->tv_manager()->dynamics<ctv::dynamics_type::ekPOPULATION>()) {
    collect("tv::population",
            *loop->tv_manager()->dynamics<ctv::dynamics_type::ekPOPULATION>());
  }
} /* collect_from_loop() */

void base_metrics_aggregator::collect_from_block(
    const crepr::base_block2D* const block) {
  collect("blocks::transport", *block->md());
} /* collect_from_block() */

void base_metrics_aggregator::collect_from_controller(
    const controller::foraging_controller* const controller) {
  collect("swarm::spatial_dist2D::pos", *controller);
} /* collect_from_controller() */

NS_END(metrics, fordyca);
