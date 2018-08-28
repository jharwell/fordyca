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
#include "fordyca/fsm/cell2D_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cell2D_fsm::cell2D_fsm(const std::shared_ptr<rcppsw::er::server> server)
    : state_machine::simple_fsm(server, ST_MAX_STATES, ST_UNKNOWN),
      state_unknown(),
      state_empty(),
      state_block(),
      state_cache(),
    state_cache_extent() {
  /* if (ERROR == attmod("cell2D_fsm")) { */
  /*   client::insmod("cell2D_fsm", */
  /*                     rcppsw::er::er_lvl::NOM, */
  /*                     rcppsw::er::er_lvl::NOM); */
  /* } */
}

cell2D_fsm::cell2D_fsm(void)
    : state_machine::simple_fsm(ST_MAX_STATES, ST_UNKNOWN),
      state_unknown(),
      state_empty(),
      state_block(),
    state_cache(),
    state_cache_extent(),
      m_block_count(0) {
  /* if (ERROR == attmod("cell2D_fsm")) { */
  /*   client::insmod("cell2D_fsm", */
  /*                     rcppsw::er::er_lvl::NOM, */
  /*                     rcppsw::er::er_lvl::NOM); */
  /* } */
}

/*******************************************************************************
 * Event Functions
 ******************************************************************************/
void cell2D_fsm::event_unknown(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    ST_UNKNOWN, /* unknown */
        ST_UNKNOWN, /* empty */
        ST_UNKNOWN, /* has block */
        ST_UNKNOWN,  /* has cache */
        ST_UNKNOWN, /* cache extent */
        };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, ST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_unknown() */

void cell2D_fsm::event_empty(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    ST_EMPTY, /* unknown */
        ST_EMPTY, /* empty */
        ST_EMPTY, /* has block */
        ST_EMPTY, /* has cache */
        ST_EMPTY, /* cache extent */
        };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, ST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_empty() */

void cell2D_fsm::event_block_drop(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    ST_HAS_BLOCK, /* unknown */
        ST_HAS_BLOCK, /* empty */
        ST_HAS_CACHE, /* has block */
        ST_HAS_CACHE,  /* has cache */
        state_machine::event_signal::FATAL, /* cache extent */
        };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, ST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()],
                 rcppsw::make_unique<block_data>(false));
} /* event_empty() */

void cell2D_fsm::event_block_pickup(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    state_machine::event_signal::FATAL, /* unknown */
        state_machine::event_signal::FATAL, /* empty */
        ST_EMPTY,                           /* has block */
        ST_HAS_CACHE,                        /* has cache */
        state_machine::event_signal::FATAL, /* cache extent */
        };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, ST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()],
                 rcppsw::make_unique<block_data>(true));
} /* event_block_pickup() */

void cell2D_fsm::event_cache_extent(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    ST_CACHE_EXTENT, /* unknown */
        ST_CACHE_EXTENT, /* empty */
        /*
         * This is technically bad, but the arena map fixes it right after
         * creating a new cache, so we can let it slide here.
         */
        ST_CACHE_EXTENT,                    /* has block */
        state_machine::event_signal::FATAL, /* has cache */
        state_machine::event_signal::FATAL, /* cache extent */
        };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, ST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_cache_extent() */

/*******************************************************************************
 * State Functions
 ******************************************************************************/
FSM_STATE_DEFINE_ND(cell2D_fsm, state_unknown) {
  if (ST_UNKNOWN != last_state()) {
    ER_DIAG("Cell in UNKNOWN state.");
    m_block_count = 0;
  }
  return state_machine::event_signal::HANDLED;
}
FSM_STATE_DEFINE_ND(cell2D_fsm, state_empty) {
  if (ST_EMPTY != last_state()) {
    ER_DIAG("Cell in EMPTY state.");
    m_block_count = 0;
  }
  return state_machine::event_signal::HANDLED;
}

FSM_STATE_DEFINE_ND(cell2D_fsm, state_block) {
  if (ST_HAS_BLOCK != last_state()) {
    m_block_count = 1;
    ER_DIAG("Cell HAS_BLOCK.");
  }
  return state_machine::event_signal::HANDLED;
}

FSM_STATE_DEFINE(cell2D_fsm, state_cache, struct block_data) {
  if (ST_HAS_CACHE != last_state()) {
    ER_DIAG("Cell HAS_CACHE.");
    ER_ASSERT(1 == m_block_count,
              "FATAL: block count should be 1 on transition to HAS_CACHE");
  }
  if (nullptr != data) {
    if (data->pickup) {
      --m_block_count;
    } else {
      ++m_block_count;
    }
  }
  if (1 == m_block_count) {
    internal_event(ST_HAS_BLOCK);
  }
  return state_machine::event_signal::HANDLED;
}
FSM_STATE_DEFINE_ND(cell2D_fsm, state_cache_extent) {
  if (ST_CACHE_EXTENT != last_state()) {
    ER_DIAG("Cell in CACHE_EXTENT state.");
    m_block_count = 0;
  }
  return state_machine::event_signal::HANDLED;
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cell2D_fsm::init(void) { state_machine::simple_fsm::init(); } /* init() */

NS_END(fsm, fordyca);
