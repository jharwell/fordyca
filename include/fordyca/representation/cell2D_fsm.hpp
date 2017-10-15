/**
 * @file cell2D_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_CELL2D_FSM_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_CELL2D_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcsw/common/common.h"
#include "rcppsw/patterns/state_machine/simple_fsm.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);
namespace state_machine = rcppsw::patterns::state_machine;
namespace visitor = rcppsw::patterns::visitor;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief Per-cell (2D square on arena map) state machine containing the current
 * state of the cell (empty, has a block, or unknown).
 */
class cell2D_fsm : public state_machine::simple_fsm,
                   public visitor::visitable<cell2D_fsm> {
 public:
  enum state {
    ST_UNKNOWN,
    ST_EMPTY,
    ST_HAS_BLOCK,
    ST_HAS_CACHE,
    ST_MAX_STATES
  };

  explicit cell2D_fsm(
      const std::shared_ptr<rcppsw::common::er_server>& server);
  virtual ~cell2D_fsm(void) {}
  bool state_is_known(void) { return current_state() != ST_UNKNOWN; }
  bool state_has_block(void) { return current_state() == ST_HAS_BLOCK; }
  bool state_has_cache(void) { return current_state() == ST_HAS_CACHE; }
  bool state_is_empty(void) { return current_state() == ST_EMPTY; }

  /* events */
  void event_unknown(void);
  void event_empty(void);
  void event_block_pickup(void);
  void event_block_drop(void);
  void init(void) override;

  uint block_count(void) const { return m_block_count; }

 private:
  struct block_data : public state_machine::event_data {
    block_data(bool pickup_ = false) : pickup(pickup_) {}
    bool pickup;
  };

  FSM_STATE_DECLARE(cell2D_fsm, state_unknown, state_machine::no_event_data);
  FSM_STATE_DECLARE(cell2D_fsm, state_empty, state_machine::no_event_data);
  FSM_STATE_DECLARE(cell2D_fsm, state_block, state_machine::no_event_data);
  FSM_STATE_DECLARE(cell2D_fsm, state_cache, struct block_data);

  FSM_DEFINE_STATE_MAP_ACCESSOR(state_map, index) override {
  FSM_DEFINE_STATE_MAP(state_map, kSTATE_MAP) {
        FSM_STATE_MAP_ENTRY(&state_unknown),
        FSM_STATE_MAP_ENTRY(&state_empty),
        FSM_STATE_MAP_ENTRY(&state_block),
        FSM_STATE_MAP_ENTRY(&state_cache),
            };
  FSM_VERIFY_STATE_MAP(state_map, kSTATE_MAP);
  return &kSTATE_MAP[index];
  }

  uint m_block_count;
};

NS_END(representation, forydca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_CELL2D_FSM_HPP_ */
