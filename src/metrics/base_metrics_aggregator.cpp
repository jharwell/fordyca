/**
 * @file base_metrics_aggregator.cpp
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
#include "fordyca/metrics/base_metrics_aggregator.hpp"
#include <experimental/filesystem>

#include "fordyca/ds/arena_map.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics_collector.hpp"
#include "fordyca/metrics/blocks/transport_metrics_collector.hpp"
#include "fordyca/metrics/fsm/collision_metrics.hpp"
#include "fordyca/metrics/fsm/collision_metrics_collector.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics_collector.hpp"
#include "fordyca/metrics/fsm/acquisition_loc_metrics_collector.hpp"
#include "fordyca/metrics/fsm/movement_metrics.hpp"
#include "fordyca/metrics/fsm/movement_metrics_collector.hpp"
#include "fordyca/metrics/robot_occupancy_metrics.hpp"
#include "fordyca/metrics/robot_occupancy_metrics_collector.hpp"
#include "fordyca/metrics/temporal_variance_metrics.hpp"
#include "fordyca/metrics/temporal_variance_metrics_collector.hpp"
#include "fordyca/params/metrics_params.hpp"
#include "fordyca/support/base_loop_functions.hpp"
#include "fordyca/support/tv/tv_manager.hpp"

#include "rcppsw/metrics/swarm/convergence_metrics.hpp"
#include "rcppsw/metrics/swarm/convergence_metrics_collector.hpp"
#include "rcppsw/swarm/convergence/convergence_calculator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace fs = std::experimental::filesystem;
NS_START(fordyca, metrics);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_metrics_aggregator::base_metrics_aggregator(
    const params::metrics_params* const mparams,
    const std::string& output_root)
    : ER_CLIENT_INIT("fordyca.metrics.base_aggregator"),
      m_metrics_path(output_root + "/" + mparams->output_dir) {
  if (!fs::exists(m_metrics_path)) {
    fs::create_directories(m_metrics_path);
  } else {
    ER_WARN("Output metrics path '%s' already exists", m_metrics_path.c_str());
  }
  register_collector<metrics::fsm::movement_metrics_collector>(
      "fsm::movement",
      metrics_path() + "/" + mparams->fsm_movement_fname,
      mparams->collect_interval);

  register_collector<metrics::fsm::collision_metrics_collector>(
      "fsm::collision",
      metrics_path() + "/" + mparams->fsm_collision_fname,
      mparams->collect_interval);

  register_collector<metrics::fsm::goal_acquisition_metrics_collector>(
      "blocks::acq_counts",
      metrics_path() + "/" + mparams->block_acq_counts_fname,
      mparams->collect_interval);
  register_collector<metrics::fsm::acquisition_loc_metrics_collector>(
      "blocks::acq_locs",
      metrics_path() + "/" + mparams->block_acq_locs_fname,
      mparams->collect_interval,
      rmath::dvec2uvec(mparams->arena_grid.upper,
                       mparams->arena_grid.resolution));

  register_collector<metrics::blocks::transport_metrics_collector>(
      "blocks::transport",
      metrics_path() + "/" + mparams->block_transport_fname,
      mparams->collect_interval);

  register_collector<metrics::blocks::manipulation_metrics_collector>(
      "blocks::manipulation",
      metrics_path() + "/" + mparams->block_manipulation_fname,
      mparams->collect_interval);

  register_collector<metrics::robot_occupancy_metrics_collector>(
      "arena::robot_locs",
      metrics_path() + "/" + mparams->arena_robot_locs_fname,
      mparams->collect_interval,
      rmath::dvec2uvec(mparams->arena_grid.upper,
                       mparams->arena_grid.resolution));

  register_collector<rmetrics::swarm::convergence_metrics_collector>(
      "swarm::convergence",
      metrics_path() + "/" + mparams->swarm_convergence_fname,
      mparams->collect_interval);

  register_collector<metrics::temporal_variance_metrics_collector>(
      "loop::temporal_variance",
      metrics_path() + "/" + mparams->temporal_variance_fname);
  reset_all();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_metrics_aggregator::collect_from_loop(
    const support::base_loop_functions* const loop) {
  collect("swarm::convergence", *loop->conv_calculator());
  collect("loop::temporal_variance", *loop->tv_manager());
} /* collect_from_loop() */

void base_metrics_aggregator::collect_from_block(
    const repr::base_block* const block) {
  collect("blocks::transport", *block);
} /* collect_from_block() */

void base_metrics_aggregator::collect_from_arena(
    const ds::arena_map* const arena) {
  collect("arena::robot_locs", *arena);
} /* collect_from_arena() */

NS_END(metrics, fordyca);
