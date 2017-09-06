/**
 * @file cell_empty.hpp
n *
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

#ifndef INCLUDE_FORDYCA_OPERATIONS_CELL_EMPTY_HPP_
#define INCLUDE_FORDYCA_OPERATIONS_CELL_EMPTY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/operations/cell_op.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation {
class cell2D_fsm;
} /* namespace representation */

NS_START(operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class cell_empty : public cell_op {
 public:
  cell_empty(void) {}

  void visit(representation::cell2D& cell);
  void visit(representation::cell2D_fsm& cell);
};

NS_END(operations, fordyca);

#endif /* INCLUDE_FORDYCA_OPERATIONS_CELL_EMPTY_HPP_ */
