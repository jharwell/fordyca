/**
 * @file block_to_cache_site_fsm.cpp
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
#include "fordyca/fsm/depth2/block_to_cache_site_fsm.hpp"
#include "fordyca/controller/saa_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth2);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_to_cache_site_fsm::block_to_cache_site_fsm(
    std::shared_ptr<rcppsw::er::server>& server,
    const controller::block_selection_matrix* bsel_matrix,
    const controller::cache_selection_matrix* csel_matrix,
    controller::saa_subsystem* const saa,
    representation::perceived_arena_map* const map)
    : block_to_goal_fsm(server, bsel_matrix, saa, map),
      m_cache_fsm(server, csel_matrix, saa, map) {}

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
acquisition_goal_type block_to_cache_site_fsm::acquisition_goal(void) const {
  if (ST_ACQUIRE_FREE_BLOCK == current_state()) {
    return block_fsm().acquisition_goal();
  } else if (ST_TRANSPORT_TO_GOAL == current_state()) {
    return m_cache_fsm.acquisition_goal();
  }
  return acquisition_goal_type::kNone;
} /* acquisition_goal() */

transport_goal_type block_to_cache_site_fsm::block_transport_goal(void) const {
  if (ST_TRANSPORT_TO_GOAL == current_state() ||
      ST_WAIT_FOR_BLOCK_DROP == current_state()) {
    return transport_goal_type::kCacheSite;
  }
  return transport_goal_type::kNone;
} /* acquisition_goal() */

NS_END(depth2, controller, fordyca);
