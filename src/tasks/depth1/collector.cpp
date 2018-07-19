/**
 * @file collector.cpp
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
#include "fordyca/tasks/depth1/collector.hpp"
#include "rcppsw/task_allocation/task_params.hpp"

#include "fordyca/controller/depth1/sensing_subsystem.hpp"
#include "fordyca/events/cache_vanished.hpp"
#include "fordyca/events/cached_block_pickup.hpp"
#include "fordyca/events/nest_block_drop.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/tasks/argument.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
collector::collector(const struct ta::task_params* const params,
                     std::unique_ptr<ta::taskable>& mechanism)
    : foraging_task(kCollectorName, params, mechanism) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void collector::task_start(const ta::taskable_argument* const) {
  foraging_signal_argument a(controller::foraging_signal::ACQUIRE_CACHED_BLOCK);
  ta::polled_task::mechanism()->task_start(&a);
  interface_complete(false);
} /* task_start() */

double collector::calc_abort_prob(void) {
  /*
   * Collectors always have a small chance of aborting their task when not at a
   * task interface. Not strictly necessary at least for now, but it IS
   * necessary for foragers and so it seems like a good idea to add this to all
   * tasks.
   */
  if (transport_goal_type::kNest == block_transport_goal()) {
    return 0.0;
  }
  return abort_prob().calc(executable_task::interface_time(),
                           executable_task::interface_estimate());
} /* calc_abort_prob() */

double collector::calc_interface_time(double start_time) {
  if (transport_goal_type::kNest == block_transport_goal() &&
      !interface_complete()) {
    interface_complete(true);
    reset_interface_time();
  }

  if (!(transport_goal_type::kNest == block_transport_goal())) {
    return current_time() - start_time;
  }
  return 0.0;
} /* calc_interface_time() */

FSM_WRAPPER_DEFINE_PTR(transport_goal_type,
                       collector,
                       block_transport_goal,
                       static_cast<fsm::depth1::cached_block_to_nest_fsm*>(
                           polled_task::mechanism()));

/*******************************************************************************
 * Event Handling
 ******************************************************************************/
void collector::accept(events::cached_block_pickup& visitor) {
  visitor.visit(*this);
}
void collector::accept(events::nest_block_drop& visitor) {
  visitor.visit(*this);
}
void collector::accept(events::cache_vanished& visitor) {
  visitor.visit(*this);
}

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
FSM_WRAPPER_DEFINE_PTR(bool,
                       collector,
                       is_avoiding_collision,
                       static_cast<fsm::depth1::cached_block_to_nest_fsm*>(
                           polled_task::mechanism()));
FSM_WRAPPER_DEFINE_PTR(bool,
                       collector,
                       is_exploring_for_goal,
                       static_cast<fsm::depth1::cached_block_to_nest_fsm*>(
                           polled_task::mechanism()));
FSM_WRAPPER_DEFINE_PTR(bool,
                       collector,
                       is_vectoring_to_goal,
                       static_cast<fsm::depth1::cached_block_to_nest_fsm*>(
                           polled_task::mechanism()));

FSM_WRAPPER_DEFINE_PTR(bool,
                       collector,
                       goal_acquired,
                       static_cast<fsm::depth1::cached_block_to_nest_fsm*>(
                           polled_task::mechanism()));

FSM_WRAPPER_DEFINE_PTR(acquisition_goal_type,
                       collector,
                       acquisition_goal,
                       static_cast<fsm::depth1::cached_block_to_nest_fsm*>(
                           polled_task::mechanism()));

/*******************************************************************************
 * Task Metrics
 ******************************************************************************/
__rcsw_pure bool collector::task_at_interface(void) const {
  return !(transport_goal_type::kNest == block_transport_goal());
} /* task_at_interface() */

NS_END(depth1, tasks, fordyca);
