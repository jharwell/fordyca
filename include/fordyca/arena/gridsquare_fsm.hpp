/**
 * @file gridsquare_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_ARENA_GRIDSQUARE_FSM_HPP_
#define INCLUDE_FORDYCA_ARENA_GRIDSQUARE_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcsw/common/common.h"
#include "rcppsw/patterns/state_machine/base_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, arena);
namespace fsm = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class gridsquare_fsm : public fsm::base_fsm {
 public:
  enum states {
    ST_UNKNOWN,
    ST_KNOWN,
    ST_EMPTY,
    ST_HAS_BLOCK,
    ST_HAS_CACHE,
    ST_MAX_STATES
  };
  enum encounter_type {
    EMPTY_GRIDSQUARE,
    BLOCK,
    CACHE
  };

  struct encounter_data : public fsm::event_data {
    enum encounter_type type;
    int cache_blocks;
  };

  explicit gridsquare_fsm(std::shared_ptr<rcppsw::common::er_server>& server) :
      base_fsm(server, ST_MAX_STATES, ST_UNKNOWN), m_cache_blocks(0) {}

  bool is_known(void) { return current_state() != ST_UNKNOWN; }
  bool has_block(void) { return current_state() == ST_HAS_BLOCK; }
  bool has_cache(void) { return current_state() == ST_HAS_CACHE; }
  bool is_empty(void) { return current_state() == ST_EMPTY; }

  /* events */
  void event_encounter(const struct encounter_data* const data);

 private:
  STATE_DECLARE(gridsquare_fsm, state_unknown, fsm::no_event_data);
  STATE_DECLARE(gridsquare_fsm, state_known, struct encounter_data);
  STATE_DECLARE(gridsquare_fsm, state_empty, fsm::no_event_data);
  STATE_DECLARE(gridsquare_fsm, state_has_block, fsm::no_event_data);
  STATE_DECLARE(gridsquare_fsm, state_has_cache, struct encounter_data);

  DEFINE_STATE_MAP_ACCESSOR(state_map) {
  DEFINE_STATE_MAP(state_map, kSTATE_MAP) {
    STATE_MAP_ENTRY(state_unknown),
        STATE_MAP_ENTRY(state_known),
        STATE_MAP_ENTRY(state_empty),
        STATE_MAP_ENTRY(state_has_block),
        STATE_MAP_ENTRY(state_has_cache)
        };
  VERIFY_STATE_MAP(state_map, kSTATE_MAP);
  return &kSTATE_MAP[0];
  }

  int m_cache_blocks;
};

NS_END(arena, forydca);
#endif /* INCLUDE_FORDYCA_ARENA_GRIDSQUARE_FSM_HPP_ */
