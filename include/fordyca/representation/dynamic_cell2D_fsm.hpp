/**
 * @file dynamic_cell2d_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_DYNAMIC_CELL2D_FSM_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_DYNAMIC_CELL2D_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcsw/common/common.h"
#include "rcppsw/patterns/state_machine/simple_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);
namespace fsm = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class dynamic_cell2D_fsm : public fsm::simple_fsm {
 public:
  enum state {
    ST_UNKNOWN,
    ST_EMPTY,
    ST_HAS_BLOCK,
    ST_HAS_CACHE,
    ST_MAX_STATES
  };
  enum new_state {
    UNKNOWN,
    EMPTY,
    BLOCK,
    CACHE
  };

  struct new_state_data : public fsm::event_data {
    explicit new_state_data(enum new_state new_state_,
                            size_t cache_blocks_ = 0) :
        new_state(new_state_), cache_blocks(cache_blocks_) {}
    enum new_state new_state;
    int cache_blocks;
  };

  explicit dynamic_cell2D_fsm(
      const std::shared_ptr<rcppsw::common::er_server>& server) :
      simple_fsm(server, ST_MAX_STATES, ST_UNKNOWN),
      state_unknown(),
      state_empty(),
      state_has_block(),
      state_has_cache(),
      m_cache_blocks(0) {}

  bool is_known(void) { return current_state() != ST_UNKNOWN; }
  bool has_block(void) { return current_state() == ST_HAS_BLOCK; }
  bool has_cache(void) { return current_state() == ST_HAS_CACHE; }
  bool is_empty(void) { return current_state() == ST_EMPTY; }

  /* events */
  void change_state(const struct new_state_data* const data);

 private:
  FSM_STATE_DECLARE(dynamic_cell2D_fsm, state_unknown, struct new_state_data);
  FSM_STATE_DECLARE(dynamic_cell2D_fsm, state_empty, struct new_state_data);
  FSM_STATE_DECLARE(dynamic_cell2D_fsm, state_has_block, struct new_state_data);
  FSM_STATE_DECLARE(dynamic_cell2D_fsm, state_has_cache, struct new_state_data);

  FSM_DEFINE_STATE_MAP_ACCESSOR(state_map) {
  FSM_DEFINE_STATE_MAP(state_map, kSTATE_MAP) {
        FSM_STATE_MAP_ENTRY(&state_unknown),
        FSM_STATE_MAP_ENTRY(&state_empty),
        FSM_STATE_MAP_ENTRY(&state_has_block),
        FSM_STATE_MAP_ENTRY(&state_has_cache)
            };
  FSM_VERIFY_STATE_MAP(state_map, kSTATE_MAP);
  return &kSTATE_MAP[0];
  }

  size_t m_cache_blocks;
};

NS_END(representation, forydca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_DYNAMIC_CELL2D_FSM_HPP_ */
