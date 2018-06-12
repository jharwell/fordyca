/**
 * @file cache_transferer.cpp
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
#include "fordyca/tasks/depth2/cache_transferer.hpp"
#include "fordyca/events/block_found.hpp"
#include "fordyca/events/cache_block_drop.hpp"
#include "fordyca/events/cache_vanished.hpp"
#include "fordyca/events/cached_block_pickup.hpp"
#include "fordyca/fsm/depth2/block_to_cache_site_fsm.hpp"
#include "fordyca/tasks/argument.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_transferer::cache_transferer(
    const struct task_allocation::task_params* params,
    std::unique_ptr<task_allocation::taskable>& mechanism)
    : foraging_task(kCacheTransfererName, params, mechanism) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

void cache_transferer::task_start(
    const task_allocation::taskable_argument* const) {
  foraging_signal_argument a(controller::foraging_signal::ACQUIRE_CACHED_BLOCK);
  task_allocation::polled_task::mechanism()->task_start(&a);
  interface_complete(false);
} /* task_start() */

double cache_transferer::calc_abort_prob(void) {
  /*
   * Cache transferers always have a small chance of aborting their task when
   * not at a task interface. Having the cache transferer task un-abortable
   * until AFTER it acquires a block from a cache can cause it to get stuck and
   * not switch to another task if it cannot find a cache anywhere.
   */
  if (transport_goal_type::kExistingCache == block_transport_goal()) {
    return abort_prob().calc(executable_task::interface_time(),
                             executable_task::interface_estimate());
  }
  return abort_prob().calc(executable_task::exec_time(),
                           executable_task::exec_estimate());
} /* calc_abort_prob() */

double cache_transferer::calc_interface_time(double start_time) {
  if (at_interface()) {
    return current_time() - start_time;
  }

  if (goal_acquired() && at_interface()) {
    if (!interface_complete()) {
      interface_complete(true);
      reset_interface_time();
    }
    return interface_time();
  }
  return 0.0;
} /* calc_interface_time() */

TASK_WRAPPER_DEFINE_PTR(transport_goal_type,
                        cache_transferer,
                        block_transport_goal,
                        static_cast<fsm::depth2::block_to_cache_site_fsm*>(
                            polled_task::mechanism()));

/*******************************************************************************
 * Event Handling
 ******************************************************************************/
void cache_transferer::accept(events::cache_block_drop& visitor) {
  visitor.visit(*this);
}

void cache_transferer::accept(events::cached_block_pickup& visitor) {
  visitor.visit(*this);
}

void cache_transferer::accept(events::cache_vanished& visitor) {
  visitor.visit(*this);
}

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
TASK_WRAPPER_DEFINE_PTR(bool,
                        cache_transferer,
                        is_avoiding_collision,
                        static_cast<fsm::depth2::block_to_cache_site_fsm*>(
                            polled_task::mechanism()));
TASK_WRAPPER_DEFINE_PTR(bool,
                        cache_transferer,
                        is_exploring_for_goal,
                        static_cast<fsm::depth2::block_to_cache_site_fsm*>(
                            polled_task::mechanism()));
TASK_WRAPPER_DEFINE_PTR(bool,
                        cache_transferer,
                        is_vectoring_to_goal,
                        static_cast<fsm::depth2::block_to_cache_site_fsm*>(
                            polled_task::mechanism()));

TASK_WRAPPER_DEFINE_PTR(bool,
                        cache_transferer,
                        goal_acquired,
                        static_cast<fsm::depth2::block_to_cache_site_fsm*>(
                            polled_task::mechanism()));

TASK_WRAPPER_DEFINE_PTR(acquisition_goal_type,
                        cache_transferer,
                        acquisition_goal,
                        static_cast<fsm::depth2::block_to_cache_site_fsm*>(
                            polled_task::mechanism()));

/*******************************************************************************
 * Task Metrics
 ******************************************************************************/
__pure bool cache_transferer::at_interface(void) const {
  return acquisition_goal_type::kExistingCache == acquisition_goal();
} /* at_interface()() */

NS_END(depth2, tasks, fordyca);
