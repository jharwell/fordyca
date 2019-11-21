/**
 * @file cell2D_states.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_FSM_CELL2D_STATES_HPP_
#define INCLUDE_FORDYCA_FSM_CELL2D_STATES_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief The states that a @ref ds::cell2D can be in.
 */
enum cell2D_states {
  /**
   * @brief The cell's contents is unknown (only used by @ref
   * depth0::mdpo_controller and its derived variants).
   */
  ekST_UNKNOWN,

  /**
   * @brief The cell is empty (does not hold a cache or block).
   */
  ekST_EMPTY,

  /**
   * @brief The cell contains a block.
   */
  ekST_HAS_BLOCK,

  /**
   * @brief The cell contains a cache.
   */
  ekST_HAS_CACHE,

  /**
   * @brief The cell does not contain a cache, but is part of the 2D space
   * occupied by a cache, in which case it also contains a reference to the
   * cache it is a part of.
   */
  ekST_CACHE_EXTENT,
  ekST_MAX_STATES
};

NS_END(fordyca, fsm);

#endif /* INCLUDE_FORDYCA_FSM_CELL2D_STATES_HPP_ */
