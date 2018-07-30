/**
 * @file stateful_metrics_aggregator.cpp
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
#include "fordyca/support/depth0/stateful_metrics_aggregator.hpp"
#include "fordyca/metrics/fsm/distance_metrics.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/metrics/world_model_metrics_collector.hpp"
#include "fordyca/params/metrics_params.hpp"

#include "fordyca/controller/depth0/stateful_foraging_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
stateful_metrics_aggregator::stateful_metrics_aggregator(
    std::shared_ptr<rcppsw::er::server> server,
    const struct params::metrics_params* params,
    const std::string& output_root)
    : stateless_metrics_aggregator(server, params, output_root) {
  register_collector<metrics::world_model_metrics_collector>(
      "perception::world_model",
      metrics_path() + "/" + params->perception_world_model_fname,
      params->collect_interval);
  reset_all();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stateful_metrics_aggregator::collect_from_controller(
    const controller::depth0::stateful_foraging_controller* const controller) {
  auto distance_m =
      dynamic_cast<const metrics::fsm::distance_metrics*>(controller);
  auto collision_m =
      dynamic_cast<const metrics::fsm::collision_metrics*>(controller);
  auto worldm_m = dynamic_cast<const metrics::world_model_metrics*>(controller);

  ER_ASSERT(distance_m,
            "FATAL: Controller does not provide FSM distance metrics");
  ER_ASSERT(worldm_m, "FATAL: Controller does not provide world model metrics");
  ER_ASSERT(collision_m,
            "FATAL: Controller does not provide FSM collision metrics");

  collect("perception::world_model", *worldm_m);
  collect("fsm::distance", *distance_m);
  collect("fsm::collision", *collision_m);

  if (controller->current_task()) {
    auto block_acq_m =
        dynamic_cast<const metrics::fsm::goal_acquisition_metrics*>(
            controller->current_task());
    ER_ASSERT(
        block_acq_m,
        "FATAL: Controller does not provide FSM block acquisition metrics");
    collect("blocks::acquisition", *block_acq_m);
  }
} /* collect_from_controller() */

NS_END(depth0, support, fordyca);
