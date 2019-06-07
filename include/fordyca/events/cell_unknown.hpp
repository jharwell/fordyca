/**
 * @file cell_unknown.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_CELL_UNKNOWN_HPP_
#define INCLUDE_FORDYCA_EVENTS_CELL_UNKNOWN_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/events/cell_op.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace ds {
class cell2D;
class occupancy_grid;
} // namespace ds
namespace fsm {
class perceived_cell2D_fsm;
}

NS_START(events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cell_unknown
 * @ingroup fordyca events detail
 *
 * @brief Created whenever a cell within an occupancy grid needs to go into an
 * unknown state.
 *
 * This happens in two cases:
 *
 * 1. After its relevance expires.
 * 2. Before the robot sees it for the first time (ala Fog of War).
 */
class cell_unknown : public cell_op, public rer::client<cell_unknown> {
 private:
  struct visit_typelist_impl {
    using inherited = cell_op::visit_typelist;
    using others = rmpl::typelist<ds::occupancy_grid>;
    using value = boost::mpl::joint_view<inherited::type, others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit cell_unknown(const rmath::vector2u& coord)
      : cell_op(coord), ER_CLIENT_INIT("fordyca.events.cell_unknown") {}

  void visit(ds::cell2D& cell);
  void visit(ds::occupancy_grid& grid);
  void visit(fsm::cell2D_fsm& fsm);
};

/**
 * @brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cell_unknown_visitor_impl =
    rpvisitor::precise_visitor<detail::cell_unknown,
                              detail::cell_unknown::visit_typelist>;

NS_END(detail);

class cell_unknown_visitor : public detail::cell_unknown_visitor_impl {
  using detail::cell_unknown_visitor_impl::cell_unknown_visitor_impl;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CELL_UNKNOWN_HPP_ */
