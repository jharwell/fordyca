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
#include "fordyca/fsm/memory_foraging_fsm.hpp"
#include "fordyca/controller/depth1_foraging_sensor_manager.hpp"
#include "fordyca/events/nest_block_drop.hpp"
#include "fordyca/events/free_block_pickup.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
double generalist::calc_elapsed_time(double exec_time) const {
  return dynamic_cast<fsm::memory_foraging_fsm*>(
      polled_task::mechanism())->sensors()->tick() - exec_time;
} /* calc_elapsed_time() */

/*******************************************************************************
 * Event Handling
 ******************************************************************************/
void generalist::accept(events::nest_block_drop &visitor) { visitor.visit(*this); }
void generalist::accept(events::free_block_pickup &visitor) { visitor.visit(*this); }

/*******************************************************************************
 * Depth0 Diagnostics
 ******************************************************************************/
bool generalist::is_searching_for_block(void) const {
  return static_cast<fsm::memory_foraging_fsm*>(
      polled_task::mechanism())->is_searching_for_block();
} /* is_searching_for_block() */

bool generalist::is_avoiding_collision(void) const {
  return static_cast<fsm::memory_foraging_fsm*>(
      polled_task::mechanism())->is_avoiding_collision();
} /* is_avoiding_collision() */

bool generalist::is_transporting_to_nest(void) const {
  return static_cast<fsm::memory_foraging_fsm*>(
      polled_task::mechanism())->is_transporting_to_nest();
} /* is_tranpsorting_to_nest() */

bool generalist::is_vectoring(void) const {
  return static_cast<fsm::memory_foraging_fsm*>(
      polled_task::mechanism())->is_vectoring();
} /* is_vectoring() */

bool generalist::is_exploring(void) const {
  return static_cast<fsm::memory_foraging_fsm*>(
      polled_task::mechanism())->is_exploring();
} /* is_exploring() */

NS_END(tasks, fordyca);
