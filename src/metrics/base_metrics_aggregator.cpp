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
#include <experimental/filesystem>

#include "rcppsw/mpl/typelist.hpp"

#include "fordyca/config/metrics_config.hpp"
#include "fordyca/config/metrics_parser.hpp"
#include "fordyca/controller/base_controller.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics_collector.hpp"
#include "fordyca/metrics/blocks/transport_metrics_collector.hpp"
#include "fordyca/metrics/collector_registerer.hpp"
#include "fordyca/metrics/temporal_variance_metrics.hpp"
#include "fordyca/metrics/temporal_variance_metrics_collector.hpp"
#include "fordyca/repr/base_block.hpp"
#include "fordyca/support/base_loop_functions.hpp"
#include "fordyca/support/tv/tv_manager.hpp"

#include "cosm/convergence/convergence_calculator.hpp"
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
#include "cosm/metrics/convergence_metrics.hpp"
#include "cosm/metrics/convergence_metrics_collector.hpp"
#include "cosm/metrics/spatial_dist2D_metrics.hpp"
#include "cosm/metrics/spatial_dist2D_pos_metrics_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace fs = std::experimental::filesystem;
NS_START(fordyca, metrics, detail);

using collector_typelist = rmpl::typelist<
    collector_registerer::type_wrap<cfmetrics::movement_metrics_collector>,
    collector_registerer::type_wrap<cfmetrics::collision_metrics_collector>,
    collector_registerer::type_wrap<cfmetrics::collision_locs_metrics_collector>,
    collector_registerer::type_wrap<cfmetrics::goal_acq_metrics_collector>,
    collector_registerer::type_wrap<cfmetrics::goal_acq_locs_metrics_collector>,
    collector_registerer::type_wrap<cfmetrics::current_explore_locs_metrics_collector>,
    collector_registerer::type_wrap<cfmetrics::current_vector_locs_metrics_collector>,
    collector_registerer::type_wrap<blocks::transport_metrics_collector>,
    collector_registerer::type_wrap<blocks::manipulation_metrics_collector>,
    collector_registerer::type_wrap<cmetrics::spatial_dist2D_pos_metrics_collector>,
    collector_registerer::type_wrap<cmetrics::convergence_metrics_collector>,
    collector_registerer::type_wrap<temporal_variance_metrics_collector> >;

NS_END(detail);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_metrics_aggregator::base_metrics_aggregator(
    const config::metrics_config* const mconfig,
    const std::string& output_root)
    : ER_CLIENT_INIT("fordyca.metrics.base_aggregator"),
      m_metrics_path(output_root + "/" + mconfig->output_dir) {
  if (!fs::exists(m_metrics_path)) {
    fs::create_directories(m_metrics_path);
  } else {
    ER_WARN("Output metrics path '%s' already exists", m_metrics_path.c_str());
  }
  collector_registerer::creatable_set creatable_set = {
      {typeid(cfmetrics::movement_metrics_collector),
       "fsm_movement",
       "fsm::movement"},
      {typeid(cfmetrics::collision_locs_metrics_collector),
       "fsm_collision_locs",
       "fsm::collision_locs"},
      {typeid(cfmetrics::collision_metrics_collector),
       "fsm_collision_counts",
       "fsm::collision_counts"},
      {typeid(cfmetrics::goal_acq_metrics_collector),
       "block_acq_counts",
       "blocks::acq_counts"},
      {typeid(cfmetrics::goal_acq_locs_metrics_collector),
       "block_acq_locs",
       "blocks::acq_locs"},
      {typeid(cfmetrics::current_explore_locs_metrics_collector),
       "block_acq_explore_locs",
       "blocks::acq_explore_locs"},
      {typeid(cfmetrics::current_vector_locs_metrics_collector),
       "block_acq_vector_locs",
       "blocks::acq_vector_locs"},
      {typeid(blocks::transport_metrics_collector),
       "block_transport",
       "blocks::transport"},
      {typeid(blocks::manipulation_metrics_collector),
       "block_manipulation",
       "blocks::manipulation"},
      {typeid(cmetrics::spatial_dist2D_pos_metrics_collector),
       "swarm_dist2D_pos",
       "swarm::spatial_dist2D::pos"},
      {typeid(cmetrics::convergence_metrics_collector),
       "swarm_convergence",
       "swarm::convergence"},
      {typeid(temporal_variance_metrics_collector),
       "loop_temporal_variance",
       "loop::temporal_variance"}};

  collector_registerer registerer(mconfig, creatable_set, this);
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
  collect("loop::temporal_variance", *loop->tv_manager());
} /* collect_from_loop() */

void base_metrics_aggregator::collect_from_block(
    const repr::base_block* const block) {
  collect("blocks::transport", *block);
} /* collect_from_block() */

void base_metrics_aggregator::collect_from_controller(
    const controller::base_controller* const controller) {
  collect("swarm::spatial_dist2D::pos", *controller);
} /* collect_from_controller() */

NS_END(metrics, fordyca);
