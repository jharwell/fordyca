/**
 * \file forager_los.cpp
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

/*******************************************************************************
 * Includes
 *****************************************************************************/
#include "fordyca/repr/forager_los.hpp"

#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/ds/cell2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
cds::entity_vector forager_los::blocks(void) const {
  cds::entity_vector blocks{};
  for (size_t i = 0; i < xsize(); ++i) {
    for (size_t j = 0; j < ysize(); ++j) {
      const cds::cell2D& cell = access(i, j);
      if (cell.state_has_block()) {
        ER_ASSERT(
            nullptr != cell.block2D() || nullptr != cell.block3D(),
            "Cell at(%zu,%zu) in HAS_BLOCK state, but does not have block",
            i,
            j);
        blocks.push_back(cell.entity());
      }
    } /* for(j..) */
  }   /* for(i..) */
  return blocks;
} /* blocks() */

cads::bcache_vectorno forager_los::caches(void) const {
  cads::bcache_vectorno caches;

  for (size_t i = 0; i < xsize(); ++i) {
    for (size_t j = 0; j < ysize(); ++j) {
      const cds::cell2D& cell = access(i, j);
      if (cell.state_has_cache() || cell.state_in_cache_extent()) {
        auto cache = cell.cache();
        ER_ASSERT(
            nullptr != cache,
            "Cell@%s in HAS_CACHE/CACHE_EXTENT state, but does not have cache",
            cell.loc().to_str().c_str());
        ER_ASSERT(cache->n_blocks() >= carepr::base_cache::kMinBlocks,
                  "Cache%d@%s has too few blocks (%zu < %zu)",
                  cache->id().v(),
                  cache->dloc().to_str().c_str(),
                  cache->n_blocks(),
                  carepr::base_cache::kMinBlocks);
        /*
         * We can't add the cache unconditionally, because cache host cells and
         * extent cells both refer to the same cache, and doing so will give you
         * double references to a single cache in a LOS, which can cause
         * problems with pheromone updating. See FORDYCA#433.
         */
        if (caches.end() == std::find(caches.begin(), caches.end(), cache)) {
          caches.push_back(cache);
        }
      }
    } /* for(j..) */
  }   /* for(i..) */

  return caches;
} /* caches() */

NS_END(repr, fordyca);
