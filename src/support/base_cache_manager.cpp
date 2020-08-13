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

#include "cosm/arena/caching_arena_map.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rtypes::spatial_dist base_cache_manager::dimension_check(
    rtypes::spatial_dist dim) const {
  double remainder = std::remainder(dim.v(), m_map->grid_resolution().v());
  if (remainder >= std::numeric_limits<double>::epsilon()) {
    ER_WARN("Reduce cache dimension %f -> %f during creation to even multiple of grid resolution %f",
            dim.v(),
            dim.v() - remainder,
            m_map->grid_resolution().v());
    dim -= remainder;
  }
  auto rdims = rmath::vector2d(dim.v(), dim.v());
  auto ddims = rmath::dvec2zvec(rdims, m_map->grid_resolution().v());
  if (RCSW_IS_EVEN(ddims.x()) || RCSW_IS_EVEN(ddims.y())) {
    ER_WARN("Reduce cache dimension %f -> %f during creation to contain odd # cells",
            dim.v(),
            dim.v() - m_map->grid_resolution().v());
    dim -= m_map->grid_resolution().v();
  }
  return dim;
}

void base_cache_manager::bloctree_update(const cads::acache_vectoro& caches) {
  for (auto &cache : caches) {
    for (auto &pair : cache->blocks()) {
      m_map->bloctree_update(pair.second,
                             carena::arena_map_locking::ekALL_HELD,
                             caches);
    } /* for(*block..) */
  } /* for(&cache..) */
} /* bloctree_update() */

NS_END(support, fordyca);
