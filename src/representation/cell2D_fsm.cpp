/**
 * @file cell2D_fsm.cpp
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
#include "fordyca/representation/cell2D_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Event Functions
 ******************************************************************************/
void cell2D_fsm::change_state(const struct new_state_data*const data) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS) {
        ST_UNKNOWN,   /* unknown */
        ST_EMPTY,     /* empty */
        ST_HAS_BLOCK, /* has block */
        ST_HAS_CACHE  /* has cache */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()],
                 std::unique_ptr<const struct new_state_data>(data));
} /* change_state() */

/*******************************************************************************
 * State Functions
 ******************************************************************************/
FSM_STATE_DEFINE(cell2D_fsm, state_unknown, struct new_state_data) {
  if (BLOCK == data->new_state) {
    internal_event(ST_HAS_BLOCK);
  } else if (CACHE == data->new_state) {
    internal_event(ST_HAS_CACHE,
                   std::unique_ptr<const struct new_state_data>(data));
  } else {
    internal_event(ST_EMPTY);
  }
  return fsm::event_signal::HANDLED;
}
FSM_STATE_DEFINE(cell2D_fsm, state_empty, struct new_state_data) {
  if (BLOCK == data->new_state) {
    internal_event(ST_HAS_BLOCK);
  } else if (CACHE == data->new_state) {
    internal_event(ST_HAS_CACHE,
                   std::unique_ptr<const struct new_state_data>(data));
  }
  return fsm::event_signal::HANDLED;
}
FSM_STATE_DEFINE(cell2D_fsm, state_has_block, struct new_state_data) {
if (CACHE == data->new_state) {
  internal_event(ST_HAS_CACHE,
                 std::unique_ptr<const struct new_state_data>(data));
} else if (EMPTY == data->new_state) {
  internal_event(ST_EMPTY);
}
  return fsm::event_signal::HANDLED;
}
FSM_STATE_DEFINE(cell2D_fsm, state_has_cache, struct new_state_data) {
  m_cache_blocks = data->cache_blocks;
  if (BLOCK == data->new_state) {
    internal_event(ST_HAS_BLOCK);
  } else if (EMPTY == data->new_state) {
    internal_event(ST_EMPTY);
  }
  return fsm::event_signal::HANDLED;
}

NS_END(representation, fordyca);
