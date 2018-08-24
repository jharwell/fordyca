/**
 * @file stateless_metrics_aggregator.cpp
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
#include "fordyca/support/depth0/stateless_metrics_aggregator.hpp"
#include "fordyca/metrics/blocks/transport_metrics_collector.hpp"
#include "fordyca/metrics/fsm/collision_metrics.hpp"
#include "fordyca/metrics/fsm/collision_metrics_collector.hpp"
#include "fordyca/metrics/fsm/movement_metrics.hpp"
#include "fordyca/metrics/fsm/movement_metrics_collector.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics_collector.hpp"
#include "fordyca/params/metrics_params.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics.hpp"
#include "fordyca/metrics/blocks/manipulation_metrics_collector.hpp"
#include "fordyca/controller/depth0/stateless_foraging_controller.hpp"
#include "fordyca/fsm/depth0/stateless_foraging_fsm.hpp"
#include "fordyca/metrics/arena_metrics.hpp"
#include "fordyca/metrics/arena_metrics_collector.hpp"
#include "fordyca/representation/arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
stateless_metrics_aggregator::stateless_metrics_aggregator(
    std::shared_ptr<rcppsw::er::server> server,
    const struct params::metrics_params* params,
    const std::string& output_root)
    : base_metrics_aggregator(server, params, output_root) {
  insmod("metrics_agg", rcppsw::er::er_lvl::DIAG, rcppsw::er::er_lvl::NOM);

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
  register_collector<metrics::arena_metrics_collector>(
      "arena::robot_occupancy",
      metrics_path() + "/" + params->arena_robot_occupancy_fname,
      params->collect_interval,
      math::rcoord_to_dcoord(params->arena_grid.upper,
                             params->arena_grid.resolution));
  reset_all();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stateless_metrics_aggregator::collect_from_controller(
    const controller::depth0::stateless_foraging_controller* controller) {
  auto collision_m =
      dynamic_cast<const metrics::fsm::collision_metrics*>(controller->fsm());
  auto mov_m =
      dynamic_cast<const metrics::fsm::movement_metrics*>(controller);
  auto block_acq_m =
      dynamic_cast<const metrics::fsm::goal_acquisition_metrics*>(controller->fsm());
  auto manip_m =
      dynamic_cast<const metrics::blocks::manipulation_metrics*>(controller);


  ER_ASSERT(mov_m,
            "FATAL: Controller does not provide FSM movement metrics");
  ER_ASSERT(block_acq_m,
            "FATAL: Controller does not provide FSM block acquisition metrics");
  ER_ASSERT(collision_m,
            "FATAL: Controller does not provide FSM collision metrics");
  ER_ASSERT(manip_m,
            "FATAL: Controller does not provide block manipulation metrics");

  collect("fsm::movement", *mov_m);
  collect("fsm::collision", *collision_m);
  collect("blocks::acquisition", *block_acq_m);
  collect("blocks::manipulation", *manip_m);
} /* collect_from_controller() */

void stateless_metrics_aggregator::collect_from_block(
    const representation::base_block* const block) {
  collect("blocks::transport", *block);
} /* collect_from_block() */

void stateless_metrics_aggregator::collect_from_arena(
    const representation::arena_map* const arena) {
  collect("arena::robot_occupancy", *arena);
} /* collect_from_arena() */

NS_END(depth0, support, fordyca);
