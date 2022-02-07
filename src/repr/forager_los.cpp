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
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
forager_los::forager_los(const rtypes::type_uuid& c_id,
                         const grid_view_type& c_view,
                         const rtypes::discretize_ratio& c_resolution)
    : grid2D_los(c_id, c_view, c_resolution),
      ER_CLIENT_INIT("fordyca.repr.forager_los") {}


/*******************************************************************************
 * Member Functions
 ******************************************************************************/
cds::block3D_vectorno forager_los::blocks(void) const {
  cds::block3D_vectorno blocks{};
  for (size_t i = 0; i < xdsize(); ++i) {
    for (size_t j = 0; j < ydsize(); ++j) {
      const auto& cell = access(i, j);
      if (cell.state_has_block() || cell.state_in_block_extent()) {
        ER_ASSERT(nullptr != cell.block3D(),
                  "Cell@%s in HAS_BLOCK/BLOCK_EXTENT state, but does not have "
                  "block",
                  rcppsw::to_string(cell.loc()).c_str());
        blocks.push_back(static_cast<crepr::sim_block3D*>(cell.entity()));
      }
    } /* for(j..) */
  } /* for(i..) */
  return blocks;
} /* blocks() */

cads::bcache_vectorno forager_los::caches(void) const {
  cads::bcache_vectorno caches;

  for (size_t i = 0; i < xdsize(); ++i) {
    for (size_t j = 0; j < ydsize(); ++j) {
      const auto& cell = access(i, j);
      if (cell.state_has_cache() || cell.state_in_cache_extent()) {
        auto* cache = cell.cache();
        ER_ASSERT(nullptr != cache,
                  "Cell@%s in HAS_CACHE/CACHE_EXTENT state, but does not have "
                  "cache",
                  cell.loc().to_str().c_str());
        ER_ASSERT(cache->n_blocks() >= carepr::base_cache::kMinBlocks,
                  "Cache%d@%s/%s has too few blocks (%zu < %zu)",
                  cache->id().v(),
                  rcppsw::to_string(cache->rcenter2D()).c_str(),
                  rcppsw::to_string(cache->dcenter2D()).c_str(),
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
  } /* for(i..) */

  return caches;
} /* caches() */

NS_END(repr, fordyca);
