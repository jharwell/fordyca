/**
 * @file harvester.cpp
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
#include "fordyca/tasks/depth1/harvester.hpp"
#include "fordyca/controller/depth1/sensing_subsystem.hpp"
#include "fordyca/events/block_found.hpp"
#include "fordyca/events/cache_block_drop.hpp"
#include "fordyca/events/cache_found.hpp"
#include "fordyca/events/cache_vanished.hpp"
#include "fordyca/events/cached_block_pickup.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/fsm/depth1/block_to_existing_cache_fsm.hpp"
#include "fordyca/tasks/argument.hpp"
#include "rcppsw/task_allocation/task_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
harvester::harvester(const struct task_allocation::task_params* params,
                     std::unique_ptr<task_allocation::taskable>& mechanism)
    : foraging_task(kHarvesterName, params, mechanism) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void harvester::task_start(const task_allocation::taskable_argument* const) {
  foraging_signal_argument a(controller::foraging_signal::ACQUIRE_FREE_BLOCK);
  task_allocation::polled_task::mechanism()->task_start(&a);
  interface_complete(false);
} /* task_start() */

double harvester::calc_abort_prob(void) {
  /*
   * Harvesters always have a small chance of aborting their task when not at a
   * task interface. Having the harvester task un-abortable until AFTER it
   * acquires a block can cause it to get stuck and not switch to another task
   * if it cannot find a block anywhere. See #232.
   */
  if (transport_goal_type::kExistingCache == block_transport_goal()) {
    return abort_prob().calc(executable_task::interface_time(),
                             executable_task::interface_estimate());
  }
  return abort_prob().calc(executable_task::exec_time(),
                           executable_task::exec_estimate());
} /* calc_abort_prob() */

double harvester::calc_interface_time(double start_time) {
  if (transport_goal_type::kExistingCache == block_transport_goal()) {
    return current_time() - start_time;
  }

  if (goal_acquired() &&
      fsm::block_transporter::goal_type::kExistingCache == block_transport_goal()) {
    if (!interface_complete()) {
      interface_complete(true);
      reset_interface_time();
    }
    return interface_time();
  }
  return 0.0;
} /* calc_interface_time() */

FSM_WRAPPER_DEFINE_PTR(transport_goal_type, harvester,
                       block_transport_goal,
                       static_cast<fsm::depth1::block_to_existing_cache_fsm*>(
                           polled_task::mechanism()));

/*******************************************************************************
 * Event Handling
 ******************************************************************************/
void harvester::accept(events::cache_block_drop& visitor) {
  visitor.visit(*this);
}
void harvester::accept(events::free_block_pickup& visitor) {
  visitor.visit(*this);
}
void harvester::accept(events::cache_vanished& visitor) {
  visitor.visit(*this);
}

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
FSM_WRAPPER_DEFINE_PTR(bool, harvester,
                       is_avoiding_collision,
                       static_cast<fsm::depth1::block_to_existing_cache_fsm*>(
                           polled_task::mechanism()));
FSM_WRAPPER_DEFINE_PTR(bool, harvester,
                       is_exploring_for_goal,
                       static_cast<fsm::depth1::block_to_existing_cache_fsm*>(
                           polled_task::mechanism()));
FSM_WRAPPER_DEFINE_PTR(bool, harvester,
                       is_vectoring_to_goal,
                       static_cast<fsm::depth1::block_to_existing_cache_fsm*>(
                           polled_task::mechanism()));

FSM_WRAPPER_DEFINE_PTR(bool, harvester,
                       goal_acquired,
                       static_cast<fsm::depth1::block_to_existing_cache_fsm*>(
                           polled_task::mechanism()));

FSM_WRAPPER_DEFINE_PTR(acquisition_goal_type, harvester,
                       acquisition_goal,
                       static_cast<fsm::depth1::block_to_existing_cache_fsm*>(
                           polled_task::mechanism()));


/*******************************************************************************
 * Task Metrics
 ******************************************************************************/
__pure bool harvester::at_interface(void) const {
  return transport_goal_type::kExistingCache == block_transport_goal();
} /* at_interface()() */

NS_END(depth1, tasks, fordyca);
