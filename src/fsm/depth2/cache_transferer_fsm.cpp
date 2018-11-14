/**
 * @file cache_transferer_fsm.cpp
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
#include "fordyca/fsm/depth2/cache_transferer_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth2);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
cache_transferer_fsm::cache_transferer_fsm(
    const controller::cache_sel_matrix* const matrix,
    controller::saa_subsystem* const saa,
    ds::perceived_arena_map* const map)
    : block_to_goal_fsm(&m_dest_cache_fsm, &m_src_cache_fsm, saa),
      m_src_cache_fsm(matrix, saa, map),
      m_dest_cache_fsm(matrix, saa, map) {}

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
acquisition_goal_type cache_transferer_fsm::acquisition_goal(void) const {
  if (ST_ACQUIRE_BLOCK == current_state() ||
      ST_WAIT_FOR_BLOCK_PICKUP == current_state() ||
      ST_WAIT_FOR_BLOCK_DROP == current_state()) {
    return acquisition_goal_type::kExistingCache;
  }
  return acquisition_goal_type::kNone;
} /* acquisition_goal() */

transport_goal_type cache_transferer_fsm::block_transport_goal(void) const {
  if (ST_ACQUIRE_BLOCK == current_state() ||
      ST_WAIT_FOR_BLOCK_PICKUP == current_state() ||
      ST_WAIT_FOR_BLOCK_DROP == current_state()) {
    return transport_goal_type::kExistingCache;
  }
  return transport_goal_type::kNone;
} /* block_transport_goal() */

bool cache_transferer_fsm::is_acquiring_dest_cache(void) const {
  return transport_goal_type::kCacheSite == block_transport_goal() &&
         m_dest_cache_fsm.task_running();
} /* is_acquiring_dest_cache() */

bool cache_transferer_fsm::is_acquiring_src_cache(void) const {
  return transport_goal_type::kCacheSite == block_transport_goal() &&
         m_src_cache_fsm.task_running();
} /* is_acquiring_src_cache() */

NS_END(depth2, fsm, fordyca);
