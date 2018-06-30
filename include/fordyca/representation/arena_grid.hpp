/**
 * @file arena_grid.hpp
 * @ingroup representation
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_ARENA_GRID_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_ARENA_GRID_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/ds/grid2D_ptr.hpp"
#include "fordyca/representation/cell2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
// namespace rcppsw { namespace er {
// class server;
// }} // namespace rcppsw::er
NS_START(fordyca, representation);
// class cell2D;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class arena_grid
 * @ingroup representation
 *
 * @brief Representation of the cells within a grid layout
 */
class arena_grid : public rcppsw::ds::grid2D_ptr<cell2D, 
                                        std::shared_ptr<rcppsw::er::server>&> {
 public:
   arena_grid(const double& resolution, size_t x_max, size_t y_max, 
       std::shared_ptr<rcppsw::er::server>& server) :
     rcppsw::ds::grid2D_ptr<cell2D, 
       std::shared_ptr<rcppsw::er::server>&>(resolution, x_max, y_max, server) {
   }

   /**
    * @brief Reset all the cells within the grid, removing all
    * references to old blocks.
    */
   void reset(void) {
     for (size_t i = 0; i < xdsize(); ++i) {
       for (size_t j = 0; j < ydsize(); ++j) {
         cell2D& cell = access(i, j);
         cell.reset();
       } /* for(j..) */
     }   /* for(i..) */
   } /* reset */
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_ARENA_GRID_HPP_ */
