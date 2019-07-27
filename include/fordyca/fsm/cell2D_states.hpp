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
enum cell2D_states {
  ekST_UNKNOWN,
  ekST_EMPTY,
  ekST_HAS_BLOCK,
  ekST_HAS_CACHE,
  ekST_CACHE_EXTENT,
  ekST_MAX_STATES
};

NS_END(fordyca, fsm);

#endif /* INCLUDE_FORDYCA_FSM_CELL2D_STATES_HPP_ */
