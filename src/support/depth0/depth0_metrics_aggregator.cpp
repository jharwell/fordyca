/**
 * @file depth0_metrics_aggregator.cpp
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
#include "fordyca/support/depth0/depth0_metrics_aggregator.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/metrics/fsm/movement_metrics.hpp"
#include "fordyca/metrics/world_model_metrics_collector.hpp"
#include "fordyca/params/metrics_params.hpp"

#include "fordyca/controller/depth0/crw_controller.hpp"
#include "fordyca/controller/depth0/dpo_controller.hpp"
#include "fordyca/controller/depth0/mdpo_controller.hpp"
#include "fordyca/ds/arena_map.hpp"
#include "fordyca/fsm/depth0/crw_fsm.hpp"
#include "fordyca/fsm/depth0/dpo_fsm.hpp"
#include "fordyca/repr/base_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);

/*******************************************************************************
 * Template Instantiations
 ******************************************************************************/
template void depth0_metrics_aggregator::collect_from_controller(
    const controller::depth0::crw_controller* const c);
template void depth0_metrics_aggregator::collect_from_controller(
    const controller::depth0::dpo_controller* const c);
template void depth0_metrics_aggregator::collect_from_controller(
    const controller::depth0::mdpo_controller* const c);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
depth0_metrics_aggregator::depth0_metrics_aggregator(
    const struct params::metrics_params* const mparams,
    const std::string& output_root)
    : base_metrics_aggregator(mparams, output_root),
      ER_CLIENT_INIT("fordyca.support.depth0.depth0_aggregator") {
  register_collector<metrics::world_model_metrics_collector>(
      "perception::world_model",
      metrics_path() + "/" + mparams->perception_world_model_fname,
      mparams->collect_interval);
  reset_all();
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
template <class T>
void depth0_metrics_aggregator::collect_from_controller(
    const T* const controller) {
  /*
   * All depth0 controllers provide these.
   */
  auto collision_m =
      dynamic_cast<const metrics::fsm::collision_metrics*>(controller->fsm());
  auto mov_m = dynamic_cast<const metrics::fsm::movement_metrics*>(controller);
  auto block_acq_m =
      dynamic_cast<const metrics::fsm::goal_acquisition_metrics*>(controller);
  auto manip_m =
      dynamic_cast<const metrics::blocks::manipulation_metrics*>(controller);

  ER_ASSERT(mov_m, "FSM does not provide movement metrics");
  ER_ASSERT(block_acq_m, "FSM does not provide block acquisition metrics");
  ER_ASSERT(collision_m, "FSM does not provide collision metrics");
  ER_ASSERT(manip_m, "FSM does not provide block manipulation metrics");

  collect("fsm::movement", *mov_m);
  collect("fsm::collision", *collision_m);
  collect("blocks::acquisition", *block_acq_m);
  collect("blocks::manipulation", *manip_m);

  /*
   * Only MDPO provides these.
   */
  auto worldm_m = dynamic_cast<const metrics::world_model_metrics*>(controller);
  if (nullptr != worldm_m) {
    collect("perception::world_model", *worldm_m);
  }
} /* collect_from_controller() */

NS_END(depth0, support, fordyca);
