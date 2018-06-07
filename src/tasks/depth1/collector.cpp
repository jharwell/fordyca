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
collector::collector(const struct task_allocation::task_params* const params,
                     std::unique_ptr<task_allocation::taskable>& mechanism)
    : polled_task(kCollectorName, params, mechanism),
      foraging_task(kCollectorName),
      m_abort_prob(&params->abort) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
__pure double collector::current_time(void) const {
  return dynamic_cast<fsm::depth1::cached_block_to_nest_fsm*>(
             polled_task::mechanism())
      ->base_sensors()
      ->tick();
} /* current_time() */

void collector::task_start(const task_allocation::taskable_argument* const) {
  foraging_signal_argument a(controller::foraging_signal::ACQUIRE_CACHED_BLOCK);
  task_allocation::polled_task::mechanism()->task_start(&a);
  m_interface_complete = false;
} /* task_start() */

double collector::calc_abort_prob(void) {
  /*
   * Collectors always have a small chance of aborting their task when not at a
   * task interface. Not strictly necessary at least for now, but it IS
   * necessary for foragers and so it seems like a good idea to add this to all
   * tasks.
   */
  if (is_transporting_to_nest()) {
    return 0.0;
  }
  return m_abort_prob.calc(executable_task::interface_time(),
                           executable_task::interface_estimate());
} /* calc_abort_prob() */

double collector::calc_interface_time(double start_time) {
  if (is_transporting_to_nest() && !m_interface_complete) {
    m_interface_complete = true;
    reset_interface_time();
  }

  if (!is_transporting_to_nest()) {
    return current_time() - start_time;
  }
  return 0.0;
} /* calc_interface_time() */

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
bool collector::is_avoiding_collision(void) const {
  return static_cast<fsm::depth1::cached_block_to_nest_fsm*>(
             polled_task::mechanism())
      ->is_avoiding_collision();
} /* is_avoiding_collision() */

bool collector::is_transporting_to_nest(void) const {
  return static_cast<fsm::depth1::cached_block_to_nest_fsm*>(
             polled_task::mechanism())
      ->is_transporting_to_nest();
} /* is_transporting_to_nest() */

bool collector::is_exploring_for_goal(void) const {
  return static_cast<fsm::depth1::cached_block_to_nest_fsm*>(
             polled_task::mechanism())
      ->is_exploring_for_goal();
} /* is_exploring_for_goal() */

bool collector::is_vectoring_to_goal(void) const {
  return static_cast<fsm::depth1::cached_block_to_nest_fsm*>(
             polled_task::mechanism())
      ->is_vectoring_to_goal();
} /* is_vectoring_to_goal() */

bool collector::goal_acquired(void) const {
  return static_cast<fsm::depth1::cached_block_to_nest_fsm*>(
      polled_task::mechanism())
      ->goal_acquired();
} /* cache_acquired() */

/*******************************************************************************
 * Task Metrics
 ******************************************************************************/
__pure bool collector::at_interface(void) const {
  return !is_transporting_to_nest();
} /* at_interface() */

NS_END(depth1, tasks, fordyca);
