/**
 * @file op_filter_status.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_TV_OP_FILTER_STATUS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TV_OP_FILTER_STATUS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief Contains the various statuses relating to robots and block operations
 * (picking up, dropping).
 */
enum class op_filter_status {
  /**
   * @brief The robot has passed all necessary filter checkes for the requested
   * operation.
   *
   */
  ekSATISFIED,

  /**
   * @brief The robot has not currently achieved the necessary internal state
   * for the block operation.
   */
  ekROBOT_INTERNAL_UNREADY,

  /**
   * @brief The robot has achieved the necessary internal state for the block
   * operation, but is not actually on a block, so the desired operation is
   * invalid.
   */
  ekROBOT_NOT_ON_BLOCK,

  /**
   * @brief The robot has requested an action that while too close to an
   * existing block in the arena.
   */
  ekBLOCK_PROXIMITY,

  /**
   * @brief The robot has requested an action while too close to an existing
   * cache in the arena.
   */
  ekCACHE_PROXIMITY
};

NS_END(tv, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TV_OP_FILTER_STATUS_HPP_ */
