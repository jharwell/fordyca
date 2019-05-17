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

#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/events/block_vanished.hpp"
#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/events/nest_block_drop.hpp"
#include "fordyca/fsm/depth0/free_block_to_nest_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
generalist::generalist(const rta::config::task_alloc_config* const config,
                       std::unique_ptr<rta::taskable> mechanism)
    : foraging_task(kGeneralistName, config, std::move(mechanism)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
__rcsw_pure double generalist::abort_prob_calc(void) {
  return executable_task::abort_prob();
} /* abort_prob_calc() */

__rcsw_pure double generalist::current_time(void) const {
  return dynamic_cast<fsm::depth0::free_block_to_nest_fsm*>(
             polled_task::mechanism())
      ->sensors()
      ->tick();
} /* current_time() */

/*******************************************************************************
 * Event Handling
 ******************************************************************************/
void generalist::accept(events::detail::nest_block_drop& visitor) {
  visitor.visit(*this);
}
void generalist::accept(events::detail::free_block_pickup& visitor) {
  visitor.visit(*this);
}
void generalist::accept(events::detail::block_vanished& visitor) {
  visitor.visit(*this);
}

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
TASK_WRAPPER_DEFINE_PTR(
    bool,
    generalist,
    is_exploring_for_goal,
    static_cast<fsm::depth0::free_block_to_nest_fsm*>(polled_task::mechanism()),
    const);
TASK_WRAPPER_DEFINE_PTR(
    bool,
    generalist,
    is_vectoring_to_goal,
    static_cast<fsm::depth0::free_block_to_nest_fsm*>(polled_task::mechanism()),
    const);

TASK_WRAPPER_DEFINE_PTR(
    bool,
    generalist,
    goal_acquired,
    static_cast<fsm::depth0::free_block_to_nest_fsm*>(polled_task::mechanism()),
    const);

TASK_WRAPPER_DEFINE_PTR(
    acquisition_goal_type,
    generalist,
    acquisition_goal,
    static_cast<fsm::depth0::free_block_to_nest_fsm*>(polled_task::mechanism()),
    const);

TASK_WRAPPER_DEFINE_PTR(
    transport_goal_type,
    generalist,
    block_transport_goal,
    static_cast<fsm::depth0::free_block_to_nest_fsm*>(polled_task::mechanism()),
    const);

TASK_WRAPPER_DEFINE_PTR(
    rmath::vector2u,
    generalist,
    acquisition_loc,
    static_cast<fsm::depth0::free_block_to_nest_fsm*>(polled_task::mechanism()),
    const);

TASK_WRAPPER_DEFINE_PTR(
    rmath::vector2u,
    generalist,
    current_explore_loc,
    static_cast<fsm::depth0::free_block_to_nest_fsm*>(polled_task::mechanism()),
    const);

TASK_WRAPPER_DEFINE_PTR(
    rmath::vector2u,
    generalist,
    current_vector_loc,
    static_cast<fsm::depth0::free_block_to_nest_fsm*>(polled_task::mechanism()),
    const);

NS_END(depth0, tasks, fordyca);
