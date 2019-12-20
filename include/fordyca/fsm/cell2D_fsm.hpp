/**
 * \file cell2D_fsm.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include <string>

#include "rcsw/common/common.h"

#include "rcppsw/patterns/fsm/simple_fsm.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/fsm/cell2D_states.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cell2D_fsm
 * \ingroup fsm
 *
 * \brief Per-cell FSM containing the current state of the cell (empty, has a
 * block, has a cache, or unknown, etc.).
 *
 */
class cell2D_fsm final : public rpfsm::simple_fsm,
                         public rer::client<cell2D_fsm> {
 public:
  using states = cell2D_states;

  cell2D_fsm(void);

  ~cell2D_fsm(void) override = default;
  cell2D_fsm& operator=(const cell2D_fsm&) = delete;

  /**
   * \brief Initialize a COPY of a class instance via copy construction.
   *
   * This function is necessary to use this class with \ref
   * rcppsw::ds::stacked_grid, because the boost::multi_array it is built on
   * only calls the constructor for the cell object type ONCE, and then uses the
   * copy constructor to initialize the rest of the cells.
   *
   * My FSM paradigm uses *MEMBER* function pointers, so you need to initialize
   * the state map cleanly WITHOUT copy construction (even though this is the
   * copy constructor), otherwise all copies of the object will use the \p other
   * object's state map (default behavior in default copy constructor). If \p
   * other is destructed, then you will get a segfault due to dangling pointers.
   */
  cell2D_fsm(const cell2D_fsm& other);

  /* simple_fsm overrides */
  void init(void) override;

  bool state_is_known(void) const {
    return current_state() !=
           std::underlying_type<states>::type(states::ekST_UNKNOWN);
  }
  bool state_has_block(void) const {
    return current_state() ==
           std::underlying_type<states>::type(states::ekST_HAS_BLOCK);
  }
  bool state_has_cache(void) const {
    return current_state() ==
           std::underlying_type<states>::type(states::ekST_HAS_CACHE);
  }
  bool state_in_cache_extent(void) const {
    return current_state() ==
           std::underlying_type<states>::type(states::ekST_CACHE_EXTENT);
  }
  bool state_is_empty(void) const {
    return current_state() ==
           std::underlying_type<states>::type(states::ekST_EMPTY);
  }

  /* events */
  void event_unknown(void);
  void event_empty(void);
  void event_block_pickup(void);
  void event_block_drop(void);
  void event_cache_extent(void);

  size_t block_count(void) const { return m_block_count; }

 private:
  struct block_data final : public rpfsm::event_data {
    explicit block_data(bool pickup_in) : pickup(pickup_in) {}
    bool pickup;
  };

  FSM_STATE_DECLARE_ND(cell2D_fsm, state_unknown);
  FSM_STATE_DECLARE_ND(cell2D_fsm, state_empty);
  FSM_STATE_DECLARE_ND(cell2D_fsm, state_block);
  FSM_STATE_DECLARE(cell2D_fsm, state_cache, struct block_data);
  FSM_STATE_DECLARE_ND(cell2D_fsm, state_cache_extent);

  FSM_DEFINE_STATE_MAP_ACCESSOR(state_map, index) override {
    return &mc_state_map[index];
  }

  FSM_DECLARE_STATE_MAP(state_map, mc_state_map, states::ekST_MAX_STATES);
  /* clang-format off */
  uint m_block_count{0};
  /* clang-format on */
};

NS_END(fsm, forydca);

#endif /* INCLUDE_FORDYCA_FSM_CELL2D_FSM_HPP_ */
