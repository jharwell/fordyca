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
#include "fordyca/metrics/fsm/movement_metrics.hpp"
#include "fordyca/metrics/fsm/movement_metrics_collector.hpp"
#include "fordyca/metrics/robot_occupancy_metrics.hpp"
#include "fordyca/metrics/robot_occupancy_metrics_collector.hpp"
#include "fordyca/params/metrics_params.hpp"
#include "fordyca/support/base_loop_functions.hpp"
#include "rcppsw/metrics/swarm/convergence_metrics.hpp"
#include "rcppsw/metrics/swarm/convergence_metrics_collector.hpp"
#include "fordyca/metrics/temporal_variance_metrics.hpp"
#include "fordyca/metrics/temporal_variance_metrics_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace fs = std::experimental::filesystem;
NS_START(fordyca, metrics);
namespace rmetrics = rcppsw::metrics;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_metrics_aggregator::base_metrics_aggregator(
    const struct params::metrics_params* params,
    const std::string& output_root)
    : ER_CLIENT_INIT("fordyca.metrics.base_aggregator"),
      collector_group(),
      m_metrics_path(output_root + "/" + params->output_dir) {
  if (!fs::exists(m_metrics_path)) {
    fs::create_directories(m_metrics_path);
  } else {
    ER_WARN("Output metrics path '%s' already exists", m_metrics_path.c_str());
  }
  register_collector<metrics::fsm::movement_metrics_collector>(
      "fsm::movement",
      metrics_path() + "/" + params->fsm_movement_fname,
      params->collect_interval);

  register_collector<metrics::fsm::collision_metrics_collector>(
      "fsm::collision",
      metrics_path() + "/" + params->fsm_collision_fname,
      params->collect_interval);

  register_collector<metrics::fsm::goal_acquisition_metrics_collector>(
      "blocks::acquisition",
      metrics_path() + "/" + params->block_acquisition_fname,
      params->collect_interval);

  register_collector<metrics::blocks::transport_metrics_collector>(
      "blocks::transport",
      metrics_path() + "/" + params->block_transport_fname,
      params->collect_interval);

  register_collector<metrics::blocks::manipulation_metrics_collector>(
      "blocks::manipulation",
      metrics_path() + "/" + params->block_manipulation_fname,
      params->collect_interval);

  register_collector<metrics::robot_occupancy_metrics_collector>(
      "arena::robot_occupancy",
      metrics_path() + "/" + params->arena_robot_occupancy_fname,
      params->collect_interval,
      rmath::dvec2uvec(params->arena_grid.upper, params->arena_grid.resolution));

  double max_dist = (params->arena_grid.lower - params->arena_grid.upper).length();
  register_collector<rmetrics::swarm::convergence_metrics_collector>(
      "swarm::convergence",
      metrics_path() + "/" + params->swarm_convergence_fname,
      params->collect_interval,
      params->pos_entropy.enable,
      rmath::ranged(0.0, max_dist),
      max_dist / params->pos_entropy.n_iterations);

  register_collector<metrics::temporal_variance_metrics_collector>(
      "loop::temporal_variance",
      metrics_path() + "/" + params->loop_temporal_variance_fname);
  reset_all();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_metrics_aggregator::collect_from_loop(
    const support::base_loop_functions* const loop) {
  collect("swarm::convergence", *loop);
  collect("loop::temporal_variance", *loop);
} /* collect_from_loop() */

void base_metrics_aggregator::collect_from_block(
    const representation::base_block* const block) {
  collect("blocks::transport", *block);
} /* collect_from_block() */

void base_metrics_aggregator::collect_from_arena(
    const ds::arena_map* const arena) {
  collect("arena::robot_occupancy", *arena);
} /* collect_from_arena() */

NS_END(metrics, fordyca);
