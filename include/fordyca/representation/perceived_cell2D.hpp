/**
 * @file perceived_cell2D.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_CELL2D_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_CELL2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>
#include <utility>
#include "rcppsw/common/common.hpp"
#include "fordyca/representation/cell2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief Per-robot representation of a cell on the 2D grid. The fsm er_server
 * is disabled, so you can't use any of the standard event reporting macros
 * without modifying \ref grid2D.
 */
/* class perceived_cell2D: public cell2D { */
/*  public: */

/*   perceived_cell2D(void) : cell2D(), m_relevance(0.0), m_delta(0.0) {} */
/*   void delta(double delta) { m_delta = delta; } */

/*   /\** */
/*    * @brief Update the relevance/freshness of the information about the state of */
/*    * the current square. */
/*    * */
/*    * Each call to this function increases the relevance of the information by a */
/*    * factor of 1.0. */
/*    *\/ */
/*   void update_relevance(void); */

/*   /\** */
/*    * @brief If another robot reports that a block is in the same state that the */
/*    * parent robot has it in, then that information is reinforced by calling \ref */
/*    * encounter. If the remote robot has it in a different state, then the */
/*    * relevance is reset to 1.0, as that means that this robot's information was */
/*    * out of date (this function called AFTER each robot has already made all the */
/*    * updates to its own internal view of the world). */
/*    * */
/*    * @param type The encounter type. */
/*    * @param cache_blocks # of blocks in the observed cache (if relevant). */
/*    *\/ */
/*   /\* void remote_encounter(cell2D_fsm::new_state state, *\/ */
/*   /\*                       int cache_blocks = 0); *\/ */
/*   /\* void encounter(cell2D_fsm::new_state state, int cache_blocks = 0); *\/ */

/*  private: */
/*   double m_relevance; */
/*   double m_delta; */
/* }; */

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_CELL2D_HPP_ */
