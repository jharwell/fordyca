/**
 * @file discrete_coord.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/representation/discrete_coord.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 *  Functions
 ******************************************************************************/
discrete_coord real_to_discrete_coord(const argos::CVector2& r_coord,
                                      double resolution) {
  return discrete_coord(r_coord.GetX() / resolution ,
                        r_coord.GetY() / resolution);
} /* real_to_discrete_coord() */

argos::CVector2 discrete_to_real_coord(const discrete_coord& d_coord,
                                      double resolution) {
  return argos::CVector2(d_coord.first * resolution ,
                        d_coord.second * resolution);
} /* real_to_discrete_coord() */

NS_END(representation, fordyca);
