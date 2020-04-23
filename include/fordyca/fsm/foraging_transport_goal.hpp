/**
 * \file foraging_transport_goal.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_FSM_FORAGING_TRANSPORT_GOAL_HPP_
#define INCLUDE_FORDYCA_FSM_FORAGING_TRANSPORT_GOAL_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \brief Represents the different locations/entities that a robot can transport
 * a block to during foraging once they have picked one up.
 */
enum class foraging_transport_goal {
  /**
   * \brief No goal--robot is probably not carrying a block.
   */
  ekNONE = -1,

  /**
   * \brief A robot has acquired a block and is currently taking it back to
   * the nest.
   */
  ekNEST,

  /**
   * \brief A robot is currently transporting an acquired block to its
   * existing cache of choice.
   */
  ekEXISTING_CACHE,

  /**
   * \brief A robot is currently transporting an acquired block to its new
   * cache of choice.
   */
  ekNEW_CACHE,

  /**
   * \brief A robot is currently transporting an acquired block to its cache
   * site of choice.
   */
  ekCACHE_SITE
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_FORAGING_TRANSPORT_GOAL_HPP_ */
