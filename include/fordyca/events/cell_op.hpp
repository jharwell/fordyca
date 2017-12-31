/**
 * @file cell_op.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_CELL_OP_HPP_
#define INCLUDE_FORDYCA_EVENTS_CELL_OP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/visitor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;
namespace representation { class cell2D; class perceived_cell2D; }
namespace fsm { class cell2D_fsm; }

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cell_op
 *
 * @brief Non-abstract interface specifying the minimum set of classes that all
 * events that operate on cells within an occupany grid need to visit.
 *
 * Also provided are the (x, y) coordinates of the cell to which the event is
 * directed. Not all derived events may need them, but they are there.
 */
class cell_op : public visitor::visitor,
                public visitor::visit_set<representation::cell2D,
                                          representation::perceived_cell2D,
                                          fsm::cell2D_fsm> {
 public:
  cell_op(size_t x, size_t y) : m_x(x), m_y(y) {}
  ~cell_op(void) override = default;

  size_t x(void) const { return m_x; }
  size_t y(void) const { return m_y; }

 private:
  size_t m_x;
  size_t m_y;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CELL_OP_HPP_ */
