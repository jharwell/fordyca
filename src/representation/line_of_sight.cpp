/**
 * @file line_of_sight.cpp
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
 *****************************************************************************/
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/ds/cell2D.hpp"
#include "fordyca/representation/base_cache.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
ds::block_list line_of_sight::blocks(void) const {
  ds::block_list blocks{};
  for (uint i = 0; i < m_view.shape()[0]; ++i) {
    for (uint j = 0; j < m_view.shape()[1]; ++j) {
      const ds::cell2D& cell = m_view[i][j];
      if (cell.state_has_block()) {
        ER_ASSERT(nullptr != cell.block(),
                  "Cell at(%u,%u) in HAS_BLOCK state, but does not have block",
                  i,
                  j);
        blocks.push_back(cell.block());
      }
    } /* for(j..) */
  }   /* for(i..) */
  return blocks;
} /* blocks() */

ds::cache_list line_of_sight::caches(void) const {
  ds::cache_list caches = m_caches;

  for (uint i = 0; i < m_view.shape()[0]; ++i) {
    for (uint j = 0; j < m_view.shape()[1]; ++j) {
      const ds::cell2D& cell = m_view[i][j];
      if (cell.state_has_cache() || cell.state_in_cache_extent()) {
        auto cache = cell.cache();
        ER_ASSERT(nullptr != cache,
                  "Cell@%s in HAS_CACHE/CACHE_EXTENT state, but does not have cache",
                  cell.loc().to_str().c_str());
        ER_ASSERT(cache->n_blocks() >= base_cache::kMinBlocks,
                  "Cache%d@%s has too few blocks (%zu < %zu)",
                  cache->id(),
                  cache->discrete_loc().to_str().c_str(),
                  cache->n_blocks(),
                  base_cache::kMinBlocks);
        /*
         * We can't add the cache unconditionally, because cache host cells and
         * extent cells both refer to the same cache, and doing so will give you
         * double references to a single cache in a LOS, which can cause
         * problems with pheromone updating. See #433.
         */
        if (caches.end() == std::find(caches.begin(), caches.end(), cache)) {
          caches.push_back(cache);
        }
      }
    } /* for(j..) */
  }   /* for(i..) */

  return caches;
} /* caches() */

__rcsw_pure const ds::cell2D& line_of_sight::cell(uint i, uint j) const {
  return const_cast<line_of_sight*>(this)->cell(i, j);
}

bool line_of_sight::contains_loc(const rmath::vector2u& loc) const {
  for (size_t i = 0; i < xsize(); ++i) {
    for (size_t j = 0; j < ysize(); ++j) {
      if (cell(i, j).loc() == loc) {
        return true;
      }
    } /* for(j..) */
  }   /* for(i..) */
  return false;
} /* contains_loc() */

__rcsw_pure ds::cell2D& line_of_sight::cell(uint i, uint j) {
  ER_ASSERT(i < m_view.shape()[0],
            "Out of bounds X access: %u >= %lu",
            i,
            m_view.shape()[0]);
  ER_ASSERT(j < m_view.shape()[1],
            "Out of bounds Y access: %u >= %lu",
            j,
            m_view.shape()[1]);
  return m_view[i][j];
}

rmath::vector2u line_of_sight::abs_ll(void) const {
  return cell(0, 0).loc();
} /* abs_ll() */

rmath::vector2u line_of_sight::abs_ul(void) const {
  return cell(0, ysize() - 1).loc();
} /* abs_ul() */

rmath::vector2u line_of_sight::abs_lr(void) const {
  return cell(xsize() - 1, 0).loc();
} /* abs_lr() */

rmath::vector2u line_of_sight::abs_ur(void) const {
  return cell(xsize() - 1, ysize() - 1).loc();
} /* abs_ur() */

NS_END(representation, fordyca);
