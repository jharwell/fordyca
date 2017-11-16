/**
 * @file forager.cpp
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
#include "fordyca/tasks/forager.hpp"
#include "fordyca/fsm/depth1/block_to_cache_fsm.hpp"
#include "fordyca/controller/depth1/foraging_sensors.hpp"
#include "fordyca/events/cached_block_pickup.hpp"
#include "fordyca/events/cache_block_drop.hpp"
#include "fordyca/events/cache_found.hpp"
#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/events/block_found.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
double forager::calc_elapsed_time(double start_time) const {
  return dynamic_cast<fsm::depth1::block_to_cache_fsm*>(
      polled_task::mechanism())->sensors()->tick() - start_time;
} /* elapsed_time() */

double forager::calc_start_time(void) const {
  return dynamic_cast<fsm::depth1::block_to_cache_fsm*>(
      polled_task::mechanism())->sensors()->tick();
} /* calc_elapsed_time() */

/*******************************************************************************
 * Event Handling
 ******************************************************************************/
void forager::accept(events::cache_block_drop &visitor) { visitor.visit(*this); }
void forager::accept(events::free_block_pickup &visitor) { visitor.visit(*this); }

/*******************************************************************************
 * Base Diagnostics
 ******************************************************************************/
bool forager::is_exploring_for_block(void) const {
  return static_cast<fsm::depth1::block_to_cache_fsm*>(
      polled_task::mechanism())->is_exploring_for_block();
} /* is_exploring_for_block() */

bool forager::is_avoiding_collision(void) const {
  return static_cast<fsm::depth1::block_to_cache_fsm*>(
      polled_task::mechanism())->is_avoiding_collision();
} /* is_avoiding_collision() */

/*******************************************************************************
 * Depth0 Diagnostics
 ******************************************************************************/
bool forager::is_acquiring_block(void) const {
  return static_cast<fsm::depth1::block_to_cache_fsm*>(
      polled_task::mechanism())->is_acquiring_block();
} /* is_acquiring_block() */

bool forager::is_vectoring_to_block(void) const {
  return static_cast<fsm::depth1::block_to_cache_fsm*>(
      polled_task::mechanism())->is_vectoring_to_block();
} /* is_vectoring_to_block() */

/*******************************************************************************
 * Depth1 Diagnostics
 ******************************************************************************/
bool forager::is_exploring_for_cache(void) const {
  return static_cast<fsm::depth1::block_to_cache_fsm*>(
      polled_task::mechanism())->is_exploring_for_cache();
} /* is_exploring_for_cache() */

bool forager::is_vectoring_to_cache(void) const {
  return static_cast<fsm::depth1::block_to_cache_fsm*>(
      polled_task::mechanism())->is_vectoring_to_cache();
} /* is_vectoring_to_cache() */

bool forager::is_acquiring_cache(void) const {
  return static_cast<fsm::depth1::block_to_cache_fsm*>(
      polled_task::mechanism())->is_acquiring_cache();
} /* is_acquiring_cache() */

bool forager::is_transporting_to_cache(void) const {
  return static_cast<fsm::depth1::block_to_cache_fsm*>(
      polled_task::mechanism())->is_transporting_to_cache();
} /* is_transporting_to_cache() */

NS_END(tasks, fordyca);
