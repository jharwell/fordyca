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
static_cache_creator::static_cache_creator(
    ds::arena_grid* const grid,
    const std::vector<rmath::vector2d>& centers,
    rtypes::spatial_dist cache_dim)
    : base_cache_creator(grid, cache_dim),
      ER_CLIENT_INIT("fordyca.support.depth1.static_cache_creator"),
      mc_centers(centers) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
ds::cache_vector static_cache_creator::create_all(
    const ds::cache_vector& c_existing_caches,
    const ds::block_cluster_vector&,
    const ds::block_vector& c_alloc_blocks,
    rtypes::timestep t) {
  ER_DEBUG("Creating caches: alloc_blocks=[%s] (%zu)",
           rcppsw::to_string(c_alloc_blocks).c_str(),
           c_alloc_blocks.size());

  ds::cache_vector created;
  auto it = c_alloc_blocks.begin();
  for (auto& center : mc_centers) {
    auto filter = [&](const auto& c) {
      return rmath::dvec2uvec(center, grid()->resolution().v()) == c->dloc();
    };
    /* static cache already exists */
    if (c_existing_caches.end() != std::find_if(c_existing_caches.begin(),
                                                c_existing_caches.end(),
                                                filter)) {
      continue;
    }

    if (static_cast<uint>(std::distance(it, c_alloc_blocks.end())) <
        base_cache::kMinBlocks) {
      ER_WARN("Not enough blocks provided to construct cache@%s: %zu < %zu",
              center.to_str().c_str(),
              std::distance(it, c_alloc_blocks.end()),
              base_cache::kMinBlocks);
      continue;
    }
    auto it2 = it;
    std::advance(it, base_cache::kMinBlocks);
    ds::block_vector cache_i_blocks(it2, it);

    ER_INFO("Creating static cache@%s: blocks=[%s] (%zu)",
            center.to_str().c_str(),
            rcppsw::to_string(cache_i_blocks).c_str(),
            cache_i_blocks.size());
    created.push_back(create_single_cache(center, cache_i_blocks, t));
  } /* for(&center..) */
  return created;
} /* create_all() */

NS_END(depth1, support, fordyca);
