/**
 * @file cache_starter.cpp
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
#include "fordyca/tasks/depth2/cache_starter.hpp"
#include "fordyca/events/block_vanished.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/fsm/depth2/block_to_cache_site_fsm.hpp"
#include "fordyca/tasks/argument.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth2);
using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using transport_goal_type = fsm::block_transporter::goal_type;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_starter::cache_starter(const struct task_allocation::task_params* params,
                             std::unique_ptr<task_allocation::taskable> mechanism)
    : foraging_task(kCacheStarterName, params, std::move(mechanism)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_starter::task_start(const task_allocation::taskable_argument* const) {
  foraging_signal_argument a(controller::foraging_signal::ACQUIRE_FREE_BLOCK);
  task_allocation::polled_task::mechanism()->task_start(&a);
  interface_complete(false);
} /* task_start() */

double cache_starter::calc_abort_prob(void) {
  /*
   * Cache starters always have a small chance of aborting their task when not
   * at a task interface. Having the cache starter task un-abortable until AFTER
   * it acquires a block can cause it to get stuck and not switch to another
   * task if it cannot find a block anywhere.
   */
  auto* fsm = static_cast<fsm::depth2::block_to_cache_site_fsm*>(mechanism());
  if (transport_goal_type::kCacheSite == fsm->block_transport_goal()) {
    return executable_task::update_abort_prob();
  }
  return executable_task::update_abort_prob();
} /* calc_abort_prob() */

double cache_starter::calc_interface_time(double start_time) {
  if (task_at_interface()) {
    return current_time() - start_time;
  }
  auto* fsm = static_cast<fsm::depth2::block_to_cache_site_fsm*>(mechanism());
  if (fsm->goal_acquired() &&
      transport_goal_type::kCacheSite == fsm->block_transport_goal()) {
    if (!interface_complete()) {
      interface_complete(true);
      reset_interface_time();
    }
    return interface_time();
  }
  return 0.0;
} /* calc_interface_time() */

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
TASK_WRAPPER_DEFINE_PTR(bool,
                        cache_starter,
                        is_exploring_for_goal,
                        static_cast<fsm::depth2::block_to_cache_site_fsm*>(
                            polled_task::mechanism()));
TASK_WRAPPER_DEFINE_PTR(bool,
                        cache_starter,
                        is_vectoring_to_goal,
                        static_cast<fsm::depth2::block_to_cache_site_fsm*>(
                            polled_task::mechanism()));

TASK_WRAPPER_DEFINE_PTR(bool,
                        cache_starter,
                        goal_acquired,
                        static_cast<fsm::depth2::block_to_cache_site_fsm*>(
                            polled_task::mechanism()));

TASK_WRAPPER_DEFINE_PTR(acquisition_goal_type,
                        cache_starter,
                        acquisition_goal,
                        static_cast<fsm::depth2::block_to_cache_site_fsm*>(
                            polled_task::mechanism()));

TASK_WRAPPER_DEFINE_PTR(transport_goal_type,
                        cache_starter,
                        block_transport_goal,
                        static_cast<fsm::depth2::block_to_cache_site_fsm*>(
                            polled_task::mechanism()));

/*******************************************************************************
 * Event Handling
 ******************************************************************************/
void cache_starter::accept(events::free_block_drop& visitor) {
  visitor.visit(*this);
}
void cache_starter::accept(events::free_block_pickup& visitor) {
  visitor.visit(*this);
}
void cache_starter::accept(events::block_vanished& visitor) {
  visitor.visit(*this);
}

/*******************************************************************************
 * Task Metrics
 ******************************************************************************/
__rcsw_pure bool cache_starter::task_at_interface(void) const {
  auto* fsm = static_cast<fsm::depth2::block_to_cache_site_fsm*>(mechanism());
  return acquisition_goal_type::kExistingCache == fsm->acquisition_goal();
} /* task_at_interface() */

NS_END(depth2, tasks, fordyca);
