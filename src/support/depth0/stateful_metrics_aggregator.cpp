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
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/metrics/fsm/movement_metrics.hpp"
#include "fordyca/metrics/world_model_metrics_collector.hpp"
#include "fordyca/params/metrics_params.hpp"

#include "fordyca/controller/depth0/stateful_controller.hpp"
#include "fordyca/fsm/depth0/stateful_fsm.hpp"
#include "fordyca/tasks/depth0/foraging_task.hpp"
#include "rcppsw/task_allocation/polled_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);
using task0 = tasks::depth0::foraging_task;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
stateful_metrics_aggregator::stateful_metrics_aggregator(
    const struct params::metrics_params* params,
    const std::string& output_root)
    : stateless_metrics_aggregator(params, output_root),
      ER_CLIENT_INIT("fordyca.support.depth0.stateful_aggregator") {
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
    const controller::depth0::stateful_controller* const controller) {
  auto worldm_m = dynamic_cast<const metrics::world_model_metrics*>(controller);
  auto manip_m =
      dynamic_cast<const metrics::blocks::manipulation_metrics*>(controller);
  auto mov_m = dynamic_cast<const metrics::fsm::movement_metrics*>(controller);

  ER_ASSERT(nullptr != mov_m,
            "Controller does not provide FSM movement metrics");
  ER_ASSERT(nullptr != worldm_m,
            "Controller does not provide world model metrics");
  ER_ASSERT(nullptr != manip_m,
            "Controller does not provide block manipulation metrics");

  collect("perception::world_model", *worldm_m);
  collect("blocks::manipulation", *manip_m);
  collect("fsm::movement", *mov_m);

  auto collision_m =
      dynamic_cast<const metrics::fsm::collision_metrics*>(controller->fsm());
  auto block_acq_m = dynamic_cast<const metrics::fsm::goal_acquisition_metrics*>(
      controller->fsm());
  ER_ASSERT(block_acq_m,
            "Controller does not provide FSM block acquisition metrics");
  ER_ASSERT(collision_m, "FSM does not provide collision metrics");

  collect("fsm::collision", *collision_m);
  collect("blocks::acquisition", *block_acq_m);
} /* collect_from_controller() */

void stateful_metrics_aggregator::task_finish_or_abort_cb(
    const ta::polled_task* const task) {
  /*
   * Both depth1 and depth2 metrics aggregators are registered on the same
   * callback, so this function will be called for the depth2 task abort/finish
   * as well, which should be ignored.
   */
  if (!task0::task_in_depth0(task)) {
    return;
  }
  collect("tasks::execution::" + task->name(),
          dynamic_cast<const rcppsw::metrics::tasks::execution_metrics&>(*task));
} /* task_finish_or_abort_cb() */

NS_END(depth0, support, fordyca);
