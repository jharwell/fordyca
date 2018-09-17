/**
 * @file arena_grid.hpp
 * @ingroup ds
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
 * @brief Ds of the cells within a grid layout
 */
class arena_grid : public rcppsw::ds::stacked_grid<arena_layer_stack> {
 public:
  using view = rcppsw::ds::grid_view<ds::cell2D>;

  constexpr static uint kCell = 0;
  constexpr static uint kRobotOccupancy = 1;

  arena_grid(double resolution, size_t x_max, size_t y_max)
      : stacked_grid(resolution, x_max, y_max) {
    for (size_t i = 0; i < xdsize(); ++i) {
      for (size_t j = 0; j < ydsize(); ++j) {
        access<kCell>(i, j).loc(rcppsw::math::dcoord2(i, j));
      } /* for(j..) */
    }   /* for(i..) */
  }

  /**
    * @brief Reset all the cells within the grid, removing all
    * references to old blocks.
    */
  void reset(void) {
    for (size_t i = 0; i < xdsize(); ++i) {
      for (size_t j = 0; j < ydsize(); ++j) {
        cell2D& cell = access<kCell>(i, j);
        cell.reset();
      } /* for(j..) */
    }   /* for(i..) */
  }     /* reset */
};

NS_END(ds, fordyca);

#endif /* INCLUDE_FORDYCA_DS_ARENA_GRID_HPP_ */
