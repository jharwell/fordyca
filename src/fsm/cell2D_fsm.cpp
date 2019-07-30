/**
 * @file cell2D_fsm.cpp
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
#include "fordyca/fsm/cell2D_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cell2D_fsm::cell2D_fsm(void)
    : rpfsm::simple_fsm(states::ekST_MAX_STATES, states::ekST_UNKNOWN),
      ER_CLIENT_INIT("fordyca.fsm.cell2D_fsm") {}

/*******************************************************************************
 * Event Functions
 ******************************************************************************/
void cell2D_fsm::event_unknown(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
      states::ekST_UNKNOWN, /* unknown */
      states::ekST_UNKNOWN, /* empty */
      states::ekST_UNKNOWN, /* has block */
      states::ekST_UNKNOWN, /* has cache */
      states::ekST_UNKNOWN, /* cache extent */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, states::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_unknown() */

void cell2D_fsm::event_empty(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
      states::ekST_EMPTY, /* unknown */
      states::ekST_EMPTY, /* empty */
      states::ekST_EMPTY, /* has block */
      states::ekST_EMPTY, /* has cache */
      states::ekST_EMPTY, /* cache extent */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, states::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_empty() */

void cell2D_fsm::event_block_drop(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
      states::ekST_HAS_BLOCK,       /* unknown */
      states::ekST_HAS_BLOCK,       /* empty */
      states::ekST_HAS_CACHE,       /* has block */
      states::ekST_HAS_CACHE,       /* has cache */
      rpfsm::event_signal::ekFATAL, /* cache extent */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, states::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()],
                 std::make_unique<block_data>(false));
} /* event_empty() */

void cell2D_fsm::event_block_pickup(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
      rpfsm::event_signal::ekFATAL, /* unknown */
      rpfsm::event_signal::ekFATAL, /* empty */
      states::ekST_EMPTY,           /* has block */
      states::ekST_HAS_CACHE,       /* has cache */
      rpfsm::event_signal::ekFATAL, /* cache extent */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, states::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()],
                 std::make_unique<block_data>(true));
} /* event_block_pickup() */

void cell2D_fsm::event_cache_extent(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
      states::ekST_CACHE_EXTENT, /* unknown */
      states::ekST_CACHE_EXTENT, /* empty */
      /*
         * This is technically bad, but the arena map fixes it right after
         * creating a new cache, so we can let it slide here.
         */
      states::ekST_CACHE_EXTENT,    /* has block */
      rpfsm::event_signal::ekFATAL, /* has cache */
      rpfsm::event_signal::ekFATAL, /* cache extent */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, states::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_cache_extent() */

/*******************************************************************************
 * State Functions
 ******************************************************************************/
FSM_STATE_DEFINE_ND(cell2D_fsm, state_unknown) {
  if (states::ekST_UNKNOWN != last_state()) {
    m_block_count = 0;
  }
  return rpfsm::event_signal::ekHANDLED;
}
FSM_STATE_DEFINE_ND(cell2D_fsm, state_empty) {
  if (states::ekST_EMPTY != last_state()) {
    m_block_count = 0;
  }
  return rpfsm::event_signal::ekHANDLED;
}

FSM_STATE_DEFINE_ND(cell2D_fsm, state_block) {
  if (states::ekST_HAS_BLOCK != last_state()) {
    m_block_count = 1;
  }
  return rpfsm::event_signal::ekHANDLED;
}

FSM_STATE_DEFINE(cell2D_fsm, state_cache, struct block_data* data) {
  if (states::ekST_HAS_CACHE != last_state()) {
    ER_ASSERT(
        1 == m_block_count, "Incorrect block count: %u vs %u", m_block_count, 1);
  }
  if (nullptr != data) {
    if (data->pickup) {
      --m_block_count;
    } else {
      ++m_block_count;
    }
  }
  if (1 == m_block_count) {
    internal_event(states::ekST_HAS_BLOCK);
  }
  return rpfsm::event_signal::ekHANDLED;
}
FSM_STATE_DEFINE_ND(cell2D_fsm, state_cache_extent) {
  if (states::ekST_CACHE_EXTENT != last_state()) {
    m_block_count = 0;
  }
  return rpfsm::event_signal::ekHANDLED;
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cell2D_fsm::init(void) { rpfsm::simple_fsm::init(); } /* init() */

NS_END(fsm, fordyca);
