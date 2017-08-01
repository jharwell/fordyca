/**
 * @file gridsquare_fsm.cpp
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
#include "fordyca/arena/gridsquare_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, arena);

/*******************************************************************************
 * Event Functions
 ******************************************************************************/
void gridsquare_fsm::event_encounter(const struct encounter_data*const data) {
  DEFINE_TRANSITION_MAP(kTRANSITIONS) {
    ST_KNOWN,
        ST_KNOWN,
        ST_KNOWN,
        ST_KNOWN,
        ST_KNOWN
  };
  VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()], data);
}

/*******************************************************************************
 * State Functions
 ******************************************************************************/
STATE_DEFINE(gridsquare_fsm, state_known, struct encounter_data) {
  if (BLOCK == data->type) {
    internal_event(ST_HAS_BLOCK);
  } else if (CACHE == data->type) {
    internal_event(ST_HAS_CACHE, data);
  } else {
    internal_event(ST_EMPTY);
  }
};

STATE_DEFINE(gridsquare_fsm, state_unknown, fsm::no_event_data) {}
STATE_DEFINE(gridsquare_fsm, state_empty, fsm::no_event_data) {}
STATE_DEFINE(gridsquare_fsm, state_has_block, fsm::no_event_data) {}
STATE_DEFINE(gridsquare_fsm, state_has_cache, struct encounter_data) {
  m_cache_blocks = data->cache_blocks;
}

NS_END(arena, fordyca);
