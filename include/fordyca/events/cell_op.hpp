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
#include <boost/variant/static_visitor.hpp>

#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace rvisitor = rcppsw::patterns::visitor;
namespace rmath = rcppsw::math;
namespace ds {
class cell2D;
}
namespace fsm {
class cell2D_fsm;
}

NS_START(events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct cell_op_visit_set {
  using value = boost::mpl::vector<ds::cell2D, fsm::cell2D_fsm>;
};

/**
 * @class cell_op
 * @ingroup fordyca events detail
 *
 * @brief Non-abstract interface specifying the minimum set of classes that all
 * events that operate on cells within an occupany grid need to visit.
 *
 * Also provided are the (x, y) coordinates of the cell to which the event is
 * directed. Not all derived events may need them, but they are there.
 */
class cell_op {
 public:
  explicit cell_op(const rmath::vector2u& coord) : m_coord(coord) {}

  virtual ~cell_op(void) = default;

  uint x(void) const { return m_coord.x(); }
  uint y(void) const { return m_coord.y(); }

  const rmath::vector2u& coord(void) const { return m_coord; }

 private:
  /* clang-format on */
  rmath::vector2u m_coord;
  /* clang-format off */
};

NS_END(detail);

/**
 * @brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cell_op_visitor = rvisitor::precise_visitor<detail::cell_op,
                                               detail::cell_op_visit_set::value>;
NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CELL_OP_HPP_ */
