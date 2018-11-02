/**
 * @file cache_finisher.cpp
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
#include "fordyca/tasks/depth2/cache_finisher.hpp"
#include "fordyca/events/block_vanished.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/fsm/depth2/block_to_new_cache_fsm.hpp"
#include "fordyca/tasks/argument.hpp"
#include "fordyca/events/cache_proximity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth2);
using transport_goal_type = fsm::block_transporter::goal_type;
using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_finisher::cache_finisher(
    const struct ta::task_allocation_params* params,
    std::unique_ptr<task_allocation::taskable> mechanism)
    : foraging_task(kCacheFinisherName, params, std::move(mechanism)),
      ER_CLIENT_INIT("fordyca.tasks.depth1.cache_finisher") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_finisher::task_start(const task_allocation::taskable_argument* const) {
  foraging_signal_argument a(controller::foraging_signal::ACQUIRE_FREE_BLOCK);
  task_allocation::polled_task::mechanism()->task_start(&a);
} /* task_start() */

double cache_finisher::abort_prob_calc(void) {
    if (-1 == active_interface()) {
    return ta::abort_probability::kMIN_ABORT_PROB;
  } else {
    return executable_task::abort_prob();
  }
} /* abort_prob_calc() */

double cache_finisher::interface_time_calc(uint interface,double start_time) {
  ER_ASSERT(0 == interface, "Bad interface ID: %u", interface);
  return current_time() - start_time;
} /* interface_time_calc() */

void cache_finisher::active_interface_update(int) {
  auto* fsm = static_cast<fsm::depth2::block_to_new_cache_fsm*>(mechanism());

  if (fsm->goal_acquired() &&
      transport_goal_type::kNewCache == fsm->block_transport_goal()) {
    if (interface_in_prog(0)) {
      interface_exit(0);
      interface_time_mark_finish(0);
      ER_TRACE("Interface finished at timestep %f", current_time());
    }
    ER_TRACE("Interface time: %f", interface_time(0));
  } else if (transport_goal_type::kNewCache == fsm->block_transport_goal()) {
    if (!interface_in_prog(0)) {
      interface_enter(0);
      interface_time_mark_start(0);
    }
    ER_TRACE("Interface start at timestep %f", current_time());
  }
} /* active_interface_update() */

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
TASK_WRAPPER_DEFINE_PTR(bool,
                        cache_finisher,
                        is_exploring_for_goal,
                        static_cast<fsm::depth2::block_to_new_cache_fsm*>(
                            polled_task::mechanism()));
TASK_WRAPPER_DEFINE_PTR(bool,
                        cache_finisher,
                        is_vectoring_to_goal,
                        static_cast<fsm::depth2::block_to_new_cache_fsm*>(
                            polled_task::mechanism()));

TASK_WRAPPER_DEFINE_PTR(bool,
                        cache_finisher,
                        goal_acquired,
                        static_cast<fsm::depth2::block_to_new_cache_fsm*>(
                            polled_task::mechanism()));

TASK_WRAPPER_DEFINE_PTR(acquisition_goal_type,
                        cache_finisher,
                        acquisition_goal,
                        static_cast<fsm::depth2::block_to_new_cache_fsm*>(
                            polled_task::mechanism()));

TASK_WRAPPER_DEFINE_PTR(transport_goal_type,
                        cache_finisher,
                        block_transport_goal,
                        static_cast<fsm::depth2::block_to_new_cache_fsm*>(
                            polled_task::mechanism()));

/*******************************************************************************
 * Event Handling
 ******************************************************************************/
void cache_finisher::accept(events::free_block_drop& visitor) {
  visitor.visit(*this);
}
void cache_finisher::accept(events::free_block_pickup& visitor) {
  visitor.visit(*this);
}
void cache_finisher::accept(events::block_vanished& visitor) {
  visitor.visit(*this);
}
void cache_finisher::accept(events::cache_proximity& visitor) {
  visitor.visit(*this);
}

NS_END(depth2, tasks, fordyca);
