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
#include "fordyca/controller/depth1/sensing_subsystem.hpp"
#include "fordyca/events/block_found.hpp"
#include "fordyca/events/cache_block_drop.hpp"
#include "fordyca/events/cache_found.hpp"
#include "fordyca/events/cache_vanished.hpp"
#include "fordyca/events/cached_block_pickup.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/fsm/depth2/block_to_cache_site_fsm.hpp"
#include "fordyca/tasks/argument.hpp"
#include "rcppsw/task_allocation/task_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth2);
using goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_starter::cache_starter(const struct task_allocation::task_params* params,
                             std::unique_ptr<task_allocation::taskable>& mechanism)
    : polled_task(kCacheStarterName, params, mechanism),
      foraging_task(kCacheStarterName),
      m_abort_prob(&params->abort) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
__pure double cache_starter::current_time(void) const {
  return dynamic_cast<fsm::depth2::block_to_cache_site_fsm*>(polled_task::mechanism())
      ->base_sensors()
      ->tick();
} /* current_time() */

void cache_starter::task_start(const task_allocation::taskable_argument* const) {
  foraging_signal_argument a(controller::foraging_signal::ACQUIRE_FREE_BLOCK);
  task_allocation::polled_task::mechanism()->task_start(&a);
  m_interface_complete = false;
} /* task_start() */

double cache_starter::calc_abort_prob(void) {
  /*
   * Cache starters always have a small chance of aborting their task when not
   * at a task interface. Having the cache starter task un-abortable until AFTER
   * it acquires a block can cause it to get stuck and not switch to another
   * task if it cannot find a block anywhere.
   */
  if (is_transporting_to_cache()) {
    return m_abort_prob.calc(executable_task::interface_time(),
                             executable_task::interface_estimate());
  }
  return m_abort_prob.calc(executable_task::exec_time(),
                           executable_task::exec_estimate());
} /* calc_abort_prob() */

double cache_starter::calc_interface_time(double start_time) {
  if (at_interface()) {
    return current_time() - start_time;
  }

  if (goal_acquired() && goal_type::kCacheSite == goal()) {
    if (!m_interface_complete) {
      m_interface_complete = true;
      reset_interface_time();
    }
    return interface_time();
  }
  return 0.0;
} /* calc_interface_time() */

/*******************************************************************************
 * Event Handling
 ******************************************************************************/
void cache_starter::accept(events::free_block_drop& visitor) {
  visitor.visit(*this);
}

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
bool cache_starter::goal_acquired(void) const {
  return static_cast<fsm::depth2::block_to_cache_site_fsm*>(polled_task::mechanism())
      ->goal_acquired();
} /* cache_goal() */

bool cache_starter::is_exploring_for_goal(void) const {
  return static_cast<fsm::depth2::block_to_cache_site_fsm*>(polled_task::mechanism())
      ->is_exploring_for_goal();
} /* is_exploring_for_goal() */

bool cache_starter::is_avoiding_collision(void) const {
  return static_cast<fsm::depth2::block_to_cache_site_fsm*>(polled_task::mechanism())
      ->is_avoiding_collision();
} /* is_avoiding_collision() */

bool cache_starter::is_vectoring_to_goal(void) const {
  return static_cast<fsm::depth2::block_to_cache_site_fsm*>(polled_task::mechanism())
      ->is_vectoring_to_goal();
} /* is_vectoring_to_goal() */

/*******************************************************************************
 * Task Metrics
 ******************************************************************************/
__pure bool cache_starter::at_interface(void) const {
  return is_transporting_to_cache();
} /* at_interface()() */

NS_END(depth2, tasks, fordyca);
