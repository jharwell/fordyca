/**
 * @file perceived_cell_op.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_PERCEIVED_CELL_OP_HPP_
#define INCLUDE_FORDYCA_EVENTS_PERCEIVED_CELL_OP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/events/cell_op.hpp"
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;
namespace rmath = rcppsw::math;
namespace controller { namespace depth2 {
class greedy_recpart_controller;
}} // namespace controller::depth2

namespace ds {
class perceived_arena_map;
}

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class perceived_cell_op
 * @ingroup events
 *
 * @brief Non-abstract interface defining the minimum visit set for al events
 * that involve robot perception.
 */
class perceived_cell_op
    : public cell_op,
      public visitor::visit_set<controller::depth2::greedy_recpart_controller,
                                ds::perceived_arena_map> {
 public:
  explicit perceived_cell_op(const rmath::vector2u& coord)
      : cell_op(coord) {}
  ~perceived_cell_op(void) override = default;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_PERCEIVED_CELL_OP_HPP_ */
