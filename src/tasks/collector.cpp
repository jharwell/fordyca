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
#include "fordyca/tasks/collector.hpp"
#include "fordyca/fsm/block_to_nest_fsm.hpp"
#include "fordyca/controller/depth1/foraging_sensors.hpp"
#include "fordyca/events/cached_block_pickup.hpp"
#include "fordyca/events/nest_block_drop.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
double collector::calc_elapsed_time(double exec_time) const {
  return dynamic_cast<fsm::block_to_nest_fsm*>(polled_task::mechanism())->sensors()->tick() - exec_time;
} /* calc_elapsed_time() */

/*******************************************************************************
 * Event Handling
 ******************************************************************************/
void collector::accept(events::cached_block_pickup &visitor) { visitor.visit(*this); }
void collector::accept(events::nest_block_drop &visitor) { visitor.visit(*this); }

/*******************************************************************************
 * Base Diagnostics
 ******************************************************************************/
bool collector::is_avoiding_collision(void) const {
return static_cast<fsm::block_to_nest_fsm*>(
polled_task::mechanism())->is_avoiding_collision();
} /* is_avoiding_collision() */

bool collector::is_transporting_to_nest(void) const {
return static_cast<fsm::block_to_nest_fsm*>(
polled_task::mechanism())->is_transporting_to_nest();
} /* is_transporting_to_nest() */

/*******************************************************************************
 * Depth1 Diagnostics
 ******************************************************************************/
bool collector::is_exploring_for_cache(void) const {
  return static_cast<fsm::block_to_nest_fsm*>(
      polled_task::mechanism())->is_exploring_for_cache();
} /* is_exploring_for_cache() */

bool collector::is_vectoring_to_cache(void) const {
return static_cast<fsm::block_to_nest_fsm*>(
polled_task::mechanism())->is_vectoring_to_cache();
} /* is_vectoring_to_cache() */

bool collector::is_acquiring_cache(void) const {
return static_cast<fsm::block_to_nest_fsm*>(
polled_task::mechanism())->is_acquiring_cache();
} /* is_acquiring_cache() */

NS_END(tasks, fordyca);
