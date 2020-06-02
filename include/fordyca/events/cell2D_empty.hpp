/**
 * \file cell2D_empty.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_EVENTS_CELL2D_EMPTY_HPP_
#define INCLUDE_FORDYCA_EVENTS_CELL2D_EMPTY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/ds/operations/cell2D_empty.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace fordyca::ds {
class arena_map;
class occupancy_grid;
class dpo_semantic_map;
} /* namespace fordyca::ds */

NS_START(fordyca, events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cell2D_empty
 * \ingroup events detail
 *
 * \brief Created whenever a cell needs to go from some other state to being
 * empty.
 *
 * The most common example of this is when a free block is picked up, and the
 * square that the block was on is now  (probably) empty. It might not be if in
 * the same timestep a new cache is created on that same cell.
 */
class cell2D_empty : public cdops::cell2D_empty,
                     public rer::client<cell2D_empty> {
 private:
  struct visit_typelist_impl {
    using inherited = cell2D_empty::visit_typelist;
    using others =
        rmpl::typelist<ds::arena_map, ds::occupancy_grid, ds::dpo_semantic_map>;
    using value = boost::mpl::joint_view<inherited::type, others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  /* parent class visit functions */
  using cdops::cell2D_empty::visit;

  explicit cell2D_empty(const rmath::vector2z& coord)
      : cdops::cell2D_empty(coord),
        ER_CLIENT_INIT("fordyca.events.cell2D_empty") {}

  void visit(ds::occupancy_grid& grid);
  void visit(ds::dpo_semantic_map& map);
};

NS_END(detail);

/**
 * \brief We use the precise visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cell2D_empty_visitor = rpvisitor::filtered_visitor<detail::cell2D_empty>;

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CELL2D_EMPTY_HPP_ */
