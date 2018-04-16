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
#include "fordyca/tasks/generalist.hpp"

#include "fordyca/controller/depth0/sensing_subsystem.hpp"
#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/events/nest_block_drop.hpp"
#include "fordyca/fsm/depth0/stateful_foraging_fsm.hpp"
#include "rcppsw/er/server.hpp"
#include "rcppsw/task_allocation/partitionable_task_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
generalist::generalist(
    const struct task_allocation::partitionable_task_params* const params,
    std::unique_ptr<task_allocation::taskable>& mechanism)
    : partitionable_polled_task(rcppsw::er::g_server,
                                kGeneralistName,
                                params,
                                mechanism),
      foraging_task(kGeneralistName),
      m_abort_prob(params->abort_reactivity, params->abort_offset) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
__pure double generalist::current_time(void) const {
  return dynamic_cast<fsm::depth0::stateful_foraging_fsm*>(
             polled_task::mechanism())
      ->base_sensors()
      ->tick();
} /* current_time() */

bool generalist::block_acquired(void) const {
  return static_cast<fsm::depth0::stateful_foraging_fsm*>(
             polled_task::mechanism())
      ->block_acquired();
} /* cache_acquired() */

double generalist::calc_abort_prob(void) {
  /*
   * Generalists always have a small chance of aborting their task when not at a
   * task interface. Not strictly necessary at least for now, but it IS
   * necessary for foragers and so it seems like a good idea to add this to all
   * tasks.
   */
  return m_abort_prob.calc(executable_task::exec_time(),
                           executable_task::exec_estimate());
} /* calc_abort_prob() */

/*******************************************************************************
 * Event Handling
 ******************************************************************************/
void generalist::accept(events::nest_block_drop& visitor) {
  visitor.visit(*this);
}
void generalist::accept(events::free_block_pickup& visitor) {
  visitor.visit(*this);
}

/*******************************************************************************
 * Stateless Metrics
 ******************************************************************************/
bool generalist::is_exploring_for_block(void) const {
  return static_cast<fsm::depth0::stateful_foraging_fsm*>(
             polled_task::mechanism())
      ->is_exploring_for_block();
} /* is_exploring_for_block() */

bool generalist::is_avoiding_collision(void) const {
  return static_cast<fsm::depth0::stateful_foraging_fsm*>(
             polled_task::mechanism())
      ->is_avoiding_collision();
} /* is_avoiding_collision() */

bool generalist::is_transporting_to_nest(void) const {
  return static_cast<fsm::depth0::stateful_foraging_fsm*>(
             polled_task::mechanism())
      ->is_transporting_to_nest();
} /* is_tranpsorting_to_nest() */

/*******************************************************************************
 * Stateful Metrics
 ******************************************************************************/
bool generalist::is_acquiring_block(void) const {
  return static_cast<fsm::depth0::stateful_foraging_fsm*>(
             polled_task::mechanism())
      ->is_acquiring_block();
} /* is_acquiring_block() */

bool generalist::is_vectoring_to_block(void) const {
  return static_cast<fsm::depth0::stateful_foraging_fsm*>(
             polled_task::mechanism())
      ->is_vectoring_to_block();
} /* is_vectoring_to_block() */

NS_END(tasks, fordyca);
