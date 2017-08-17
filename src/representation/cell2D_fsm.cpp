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
void cell2D_fsm::event_unknown(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS) {
        ST_UNKNOWN,   /* unknown */
        ST_UNKNOWN,     /* empty */
        ST_UNKNOWN, /* has block */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()], NULL);
} /* event_unknown() */

void cell2D_fsm::event_empty(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS) {
        ST_EMPTY,   /* unknown */
        ST_EMPTY,     /* empty */
        ST_EMPTY, /* has block */
    };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()], NULL);
} /* event_empty() */

void cell2D_fsm::event_has_block(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS) {
        ST_HAS_BLOCK,   /* unknown */
        ST_HAS_BLOCK,     /* empty */
        ST_HAS_BLOCK, /* has block */
    };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()], NULL);
} /* event_has_block() */

/*******************************************************************************
 * State Functions
 ******************************************************************************/
FSM_STATE_DEFINE(cell2D_fsm, state_unknown, fsm::no_event_data) {
  return fsm::event_signal::HANDLED;
}
FSM_STATE_DEFINE(cell2D_fsm, state_empty, fsm::no_event_data) {
  ER_NOM("Cell in EMPTY state.")
  return fsm::event_signal::HANDLED;
}
FSM_STATE_DEFINE(cell2D_fsm, state_block, fsm::no_event_data) {
  ER_NOM("Cell HAS_BLOCK.")
  return fsm::event_signal::HANDLED;
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cell2D_fsm::init(void) {
  simple_fsm::init();
} /* init() */

NS_END(representation, fordyca);
