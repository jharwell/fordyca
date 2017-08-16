/**
 * @file dynamic_cell2D.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_DYNAMIC_CELL2D_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_DYNAMIC_CELL2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>
#include <utility>
#include "rcppsw/common/common.hpp"
#include "fordyca/representation/dynamic_cell2D_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief Base representation of a cell on the 2D grid. This class represents
 * the ACTUAL state of the grid (i.e. global/omniscient state).
 */
class dynamic_cell2D {
 public:
  dynamic_cell2D(void) : m_fsm(rcppsw::common::g_null_server) {}

  uint8_t state(void) const { return m_fsm.current_state(); }

 protected:
  dynamic_cell2D_fsm& fsm(void) { return m_fsm; }

 private:
  dynamic_cell2D_fsm m_fsm;
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_DYNAMIC_CELL2D_HPP_ */
