/**
 * @file static_cache_creator.cpp
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
#include "fordyca/support/depth1/static_cache_creator.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/repr/arena_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);
using repr::base_cache;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
static_cache_creator::static_cache_creator(ds::arena_grid* const grid,
                                           const std::vector<rmath::vector2d>& centers,
                                           double cache_dim)
    : base_cache_creator(grid, cache_dim),
      ER_CLIENT_INIT("fordyca.support.depth1.static_cache_creator"),
      mc_centers(centers) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
ds::cache_vector static_cache_creator::create_all(
    const ds::cache_vector& existing_caches,
    const ds::block_cluster_vector&,
    const ds::block_vector& blocks,
    uint timestep) {
  ER_DEBUG("Creating caches: free_blocks=[%s] (%zu)",
           rcppsw::to_string(blocks).c_str(),
           blocks.size());

  ds::cache_vector created;
  auto it = blocks.begin();
  for (auto &center : mc_centers) {
    auto filter = [&](const auto& c) {
      return rmath::dvec2uvec(center, grid()->resolution()) == c->dloc();
    };
    /* static cache already exists */
    if (existing_caches.end() != std::find_if(existing_caches.begin(),
                                              existing_caches.end(),
                                              filter)) {
      continue;
    }
    ER_ASSERT(static_cast<uint>(std::distance(it, blocks.end())) >=
              base_cache::kMinBlocks,
              "Not enough blocks provided to construct cache@%s: %zu < %zu",
              center.to_str().c_str(),
              std::distance(it, blocks.end()),
              base_cache::kMinBlocks);
    auto it2 = it;
    std::advance(it, base_cache::kMinBlocks);
    ds::block_list cache_i_blocks(it2, it);

    ER_INFO("Creating static cache@%s from %zu free blocks",
            center.to_str().c_str(),
            cache_i_blocks.size());
    created.push_back(create_single_cache(cache_i_blocks, center, timestep));
  } /* for(&center..) */
  return created;
} /* create_all() */

NS_END(depth1, support, fordyca);
