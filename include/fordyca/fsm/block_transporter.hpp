/**
 * @file block_transporter.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_BLOCK_TRANSPORTER_HPP_
#define INCLUDE_FORDYCA_FSM_BLOCK_TRANSPORTER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_transporter
 * @ingroup fsm
 *
 * @brief Interface defining what classes directly involved in transporting
 * blocks need to implement in order to successfully interact with the loop functions.
 */
class block_transporter {
 public:
  enum class goal_type {
    /**
     * @brief No goal--robot is probably not carrying a block.
     */
    kNone,

    /**
       * @brief A robot has acquired a block and is currently taking it back to
       * the nest.
       */
    kNest,

    /**
       * @brief A robot is currently transporting an acquired block to its
       * existing cache of choice.
       */
    kExistingCache,

    /**
       * @brief A robot is currently transporting an acquired block to its new
       * cache of choice.
       */
    kNewCache,

    /**
       * @brief A robot is currently transporting an acquired block to its cache
       * site of choice.
       */
    kCacheSite
  };
  block_transporter(void) = default;
  virtual ~block_transporter(void) = default;

  /**
   * @brief All tasks must define method to determine what they are currently
   * doing with a block (if they are carrying one).
   */
  virtual goal_type block_transport_goal(void) const = 0;
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_BLOCK_TRANSPORTER_HPP_ */
