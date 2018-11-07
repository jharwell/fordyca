/**
 * @file arena_grid.hpp
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

#ifndef INCLUDE_FORDYCA_DS_ARENA_GRID_HPP_
#define INCLUDE_FORDYCA_DS_ARENA_GRID_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <tuple>

#include "fordyca/ds/cell2D.hpp"
#include "rcppsw/ds/stacked_grid.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ds);
using arena_layer_stack = std::tuple<cell2D, bool>;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class arena_grid
 * @ingroup ds
 *
 * @brief 2D grid of \ref cell2D objects containing the state of the geometrical
 * extent of the arena floor.
 */
class arena_grid : public rcppsw::ds::stacked_grid<arena_layer_stack> {
 public:
  using view = rcppsw::ds::grid_view<ds::cell2D>;
  using const_view = rcppsw::ds::const_grid_view<ds::cell2D>;

  constexpr static uint kCell = 0;
  constexpr static uint kRobotOccupancy = 1;

  /**
   *
   * @param resolution The arena resolution (i.e. what is the size of 1 cell in
   *                   the 2D grid).
   * @param x_max      Size in X of 2D grid.
   * @param y_max      Size in Y of 2D grid.
   *
   * The origin of the grid is in the lower left corner at (0,0).
   */
  arena_grid(double resolution, size_t x_max, size_t y_max)
      : stacked_grid(resolution, x_max, y_max) {
    for (uint i = 0; i < xdsize(); ++i) {
      for (uint j = 0; j < ydsize(); ++j) {
        access<kCell>(i, j).loc(rcppsw::math::dcoord2(i, j));
      } /* for(j..) */
    }   /* for(i..) */
  }

  /**
    * @brief Reset all the cells within the grid, removing all references to old
    * blocks as well as setting all cells back to an empty state.
    */
  void reset(void) {
    for (uint i = 0; i < xdsize(); ++i) {
      for (uint j = 0; j < ydsize(); ++j) {
        access<kCell>(i, j).reset();
      } /* for(j..) */
    }   /* for(i..) */
  }     /* reset */
};

NS_END(ds, fordyca);

#endif /* INCLUDE_FORDYCA_DS_ARENA_GRID_HPP_ */
