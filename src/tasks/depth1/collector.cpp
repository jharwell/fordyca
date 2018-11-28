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

#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/events/cache_vanished.hpp"
#include "fordyca/events/cached_block_pickup.hpp"
#include "fordyca/events/nest_block_drop.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/metrics/blocks/transport_metrics.hpp"
#include "fordyca/tasks/argument.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth1);
using transport_goal_type = fsm::block_transporter::goal_type;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
collector::collector(const struct ta::task_allocation_params* const params,
                     std::unique_ptr<ta::taskable> mechanism)
    : collector{params, kCollectorName, std::move(mechanism)} {}

collector::collector(const struct ta::task_allocation_params* const params,
                     const std::string& name,
                     std::unique_ptr<ta::taskable> mechanism)
    : foraging_task(name, params, std::move(mechanism)),
      ER_CLIENT_INIT("fordyca.tasks.depth1.collector") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void collector::task_start(const ta::taskable_argument* const) {
  foraging_signal_argument a(controller::foraging_signal::ACQUIRE_CACHED_BLOCK);
  ta::polled_task::mechanism()->task_start(&a);
} /* task_start() */

double collector::abort_prob_calc(void) {
  /*
   * Collectors always have a small chance of aborting their task when not at a
   * task interface. Not strictly necessary at least for now, but it IS
   * necessary for foragers and so it seems like a good idea to add this to all
   * tasks.
   */
  if (-1 == active_interface()) {
    return ta::abort_probability::kMIN_ABORT_PROB;
  } else {
    return executable_task::abort_prob();
  }
} /* abort_prob_calc() */

double collector::interface_time_calc(uint interface, double start_time) {
  ER_ASSERT(0 == interface, "Bad interface ID: %u", interface);
  return current_time() - start_time;
} /* interface_time_calc() */

void collector::active_interface_update(int) {
  auto* fsm = static_cast<fsm::depth1::cached_block_to_nest_fsm*>(mechanism());
  if (acquisition_goal_type::kExistingCache != fsm->acquisition_goal()) {
    return;
  }

  if (!fsm->goal_acquired()) {
    if (!interface_in_prog(0)) {
      interface_enter(0);
      interface_time_mark_start(0);
      ER_TRACE("Interface start at timestep %f", current_time());
    }
  } else if (fsm->goal_acquired()) {
    if (interface_in_prog(0)) {
      interface_exit(0);
      interface_time_mark_finish(0);
      ER_TRACE("Interface finished at timestep %f", current_time());
      ER_DEBUG("Interface time: %f", interface_time(0));
    }
  }
} /* active_interface_update() */

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
TASK_WRAPPER_DEFINEC_PTR(bool,
                         collector,
                         is_exploring_for_goal,
                         static_cast<fsm::depth1::cached_block_to_nest_fsm*>(
                             polled_task::mechanism()));
TASK_WRAPPER_DEFINEC_PTR(bool,
                         collector,
                         is_vectoring_to_goal,
                         static_cast<fsm::depth1::cached_block_to_nest_fsm*>(
                             polled_task::mechanism()));

TASK_WRAPPER_DEFINEC_PTR(bool,
                         collector,
                         goal_acquired,
                         static_cast<fsm::depth1::cached_block_to_nest_fsm*>(
                             polled_task::mechanism()));

TASK_WRAPPER_DEFINEC_PTR(acquisition_goal_type,
                         collector,
                         acquisition_goal,
                         static_cast<fsm::depth1::cached_block_to_nest_fsm*>(
                             polled_task::mechanism()));

TASK_WRAPPER_DEFINEC_PTR(transport_goal_type,
                         collector,
                         block_transport_goal,
                         static_cast<fsm::depth1::cached_block_to_nest_fsm*>(
                             polled_task::mechanism()));

/*******************************************************************************
 * Task Metrics
 ******************************************************************************/
__rcsw_pure bool collector::task_at_interface(void) const {
  auto* fsm = static_cast<fsm::depth1::cached_block_to_nest_fsm*>(mechanism());
  return !(transport_goal_type::kNest == fsm->block_transport_goal());
} /* task_at_interface() */

NS_END(depth1, tasks, fordyca);
