/**
 * @file cell_empty.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_CELL_EMPTY_HPP_
#define INCLUDE_FORDYCA_EVENTS_CELL_EMPTY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

#include "fordyca/events/cell_op.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace ds {
class arena_map;
class occupancy_grid;
class dpo_semantic_map;
} // namespace ds

NS_START(events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cell_empty
 * @ingroup fordyca events detail
 *
 * @brief Created whenever a cell needs to go from some other state to being
 * empty.
 *
 * The most common example of this is when a free block is picked up, and the
 * square that the block was on is now  (probably) empty. It might not be if in
 * the same timestep a new cache is created on that same cell.
 */
class cell_empty : public cell_op, public rer::client<cell_empty> {
 private:
  struct visit_typelist_impl {
    using inherited = cell_op::visit_typelist;
    using others =
        rmpl::typelist<ds::arena_map, ds::occupancy_grid, ds::dpo_semantic_map>;
    using value = boost::mpl::joint_view<inherited::type, others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit cell_empty(const rmath::vector2u& coord)
      : cell_op(coord), ER_CLIENT_INIT("fordyca.events.cell_empty") {}

  void visit(ds::cell2D& cell);
  void visit(fsm::cell2D_fsm& fsm);
  void visit(ds::arena_map& map);
  void visit(ds::occupancy_grid& grid);
  void visit(ds::dpo_semantic_map& map);
};

/**
 * @brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cell_empty_visitor_impl =
    rpvisitor::precise_visitor<detail::cell_empty,
                               detail::cell_empty::visit_typelist>;

NS_END(detail);

class cell_empty_visitor : public detail::cell_empty_visitor_impl {
  using detail::cell_empty_visitor_impl::cell_empty_visitor_impl;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CELL_EMPTY_HPP_ */
