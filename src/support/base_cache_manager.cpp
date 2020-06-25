/**
 * \file base_cache_manager.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
#include "fordyca/support/base_cache_manager.hpp"

#include <cmath>

#include "cosm/ds/arena_grid.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rtypes::spatial_dist base_cache_manager::dimension_check(
    rtypes::spatial_dist dim) const {
  if (std::remainder(dim.v(), arena_grid()->resolution().v()) >=
      std::numeric_limits<double>::epsilon()) {
    ER_WARN("Reducing cache dimension %f -> %f during creation",
            dim.v(),
            dim.v() - arena_grid()->resolution().v());
    return dim -= arena_grid()->resolution().v();
  }
  return dim;
}

NS_END(support, fordyca);
