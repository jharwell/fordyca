/**
 * @file generalist.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/tasks/depth0/generalist.hpp"

#include "fordyca/controller/depth0/sensing_subsystem.hpp"
#include "fordyca/events/block_vanished.hpp"
#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/events/nest_block_drop.hpp"
#include "fordyca/fsm/depth0/stateful_fsm.hpp"
#include "rcppsw/task_allocation/partitionable_task_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
generalist::generalist(const struct ta::partitionable_task_params* const params,
                       std::unique_ptr<ta::taskable> mechanism)
    : partitionable_polled_task(kGeneralistName, params, std::move(mechanism)) {
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
__rcsw_pure double generalist::current_time(void) const {
  return dynamic_cast<fsm::depth0::stateful_fsm*>(polled_task::mechanism())
      ->base_sensors()
      ->tick();
} /* current_time() */

/*******************************************************************************
 * Event Handling
 ******************************************************************************/
void generalist::accept(events::nest_block_drop& visitor) {
  visitor.visit(*this);
}
void generalist::accept(events::free_block_pickup& visitor) {
  visitor.visit(*this);
}
void generalist::accept(events::block_vanished& visitor) {
  visitor.visit(*this);
}

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
TASK_WRAPPER_DEFINE_PTR(
    bool,
    generalist,
    is_exploring_for_goal,
    static_cast<fsm::depth0::stateful_fsm*>(polled_task::mechanism()));
TASK_WRAPPER_DEFINE_PTR(
    bool,
    generalist,
    is_vectoring_to_goal,
    static_cast<fsm::depth0::stateful_fsm*>(polled_task::mechanism()));

TASK_WRAPPER_DEFINE_PTR(
    bool,
    generalist,
    goal_acquired,
    static_cast<fsm::depth0::stateful_fsm*>(polled_task::mechanism()));

TASK_WRAPPER_DEFINE_PTR(
    acquisition_goal_type,
    generalist,
    acquisition_goal,
    static_cast<fsm::depth0::stateful_fsm*>(polled_task::mechanism()));

TASK_WRAPPER_DEFINE_PTR(
    transport_goal_type,
    generalist,
    block_transport_goal,
    static_cast<fsm::depth0::stateful_fsm*>(polled_task::mechanism()));

NS_END(depth0, tasks, fordyca);
