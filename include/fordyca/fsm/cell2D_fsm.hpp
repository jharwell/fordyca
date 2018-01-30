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

#ifndef INCLUDE_FORDYCA_FSM_CELL2D_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_CELL2D_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/state_machine/simple_fsm.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "rcsw/common/common.h"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

namespace state_machine = rcppsw::patterns::state_machine;
namespace visitor = rcppsw::patterns::visitor;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cell2D_fsm
 * @ingroup fsm
 *
 * @brief Per-cell FSM containing the current state of the cell (empty, has a
 * block, has a cache, or unknown).
 *
 * Though this class processes events, it does not do so via the visitor
 * pattern as it is so simple that using the visitor pattern would be overkill
 * here.
 */
class cell2D_fsm : public state_machine::simple_fsm,
                   public visitor::visitable_any<cell2D_fsm> {
 public:
  enum state {
    ST_UNKNOWN,
    ST_EMPTY,
    ST_HAS_BLOCK,
    ST_HAS_CACHE,
    ST_MAX_STATES
  };

  explicit cell2D_fsm(const std::shared_ptr<rcppsw::er::server>& server);
  cell2D_fsm(void);
  ~cell2D_fsm(void) override = default;

  bool state_is_known(void) const { return current_state() != ST_UNKNOWN; }
  bool state_has_block(void) const { return current_state() == ST_HAS_BLOCK; }
  bool state_has_cache(void) const { return current_state() == ST_HAS_CACHE; }
  bool state_is_empty(void) const { return current_state() == ST_EMPTY; }

  /* events */
  void event_unknown(void);
  void event_empty(void);
  void event_block_pickup(void);
  void event_block_drop(void);
  void init(void) override;

  size_t block_count(void) const { return m_block_count; }

 private:
  struct block_data : public state_machine::event_data {
    explicit block_data(bool pickup_) : pickup(pickup_) {}
    bool pickup;
  };

  FSM_STATE_DECLARE_ND(cell2D_fsm, state_unknown);
  FSM_STATE_DECLARE_ND(cell2D_fsm, state_empty);
  FSM_STATE_DECLARE_ND(cell2D_fsm, state_block);
  FSM_STATE_DECLARE(cell2D_fsm, state_cache, struct block_data);

  FSM_DEFINE_STATE_MAP_ACCESSOR(state_map, index) override {
    FSM_DEFINE_STATE_MAP(state_map, kSTATE_MAP){
        FSM_STATE_MAP_ENTRY(&state_unknown),
        FSM_STATE_MAP_ENTRY(&state_empty),
        FSM_STATE_MAP_ENTRY(&state_block),
        FSM_STATE_MAP_ENTRY(&state_cache),
    };
    FSM_VERIFY_STATE_MAP(state_map, kSTATE_MAP, ST_MAX_STATES);
    return &kSTATE_MAP[index];
  }

  size_t m_block_count;
};

NS_END(fsm, forydca);

#endif /* INCLUDE_FORDYCA_FSM_CELL2D_FSM_HPP_ */
