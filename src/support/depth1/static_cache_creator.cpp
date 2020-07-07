/**
 * \file static_cache_creator.cpp
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
 ******************************************************************************/
#include "fordyca/support/depth1/static_cache_creator.hpp"

#include "cosm/arena/repr/arena_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);
using carepr::base_cache;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
static_cache_creator::static_cache_creator(
    cds::arena_grid* const grid,
    const std::vector<rmath::vector2d>& centers,
    const rtypes::spatial_dist& cache_dim)
    : base_cache_creator(grid, cache_dim),
      ER_CLIENT_INIT("fordyca.support.depth1.static_cache_creator"),
      mc_centers(centers) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
static_cache_creator::creation_result static_cache_creator::create_all(
    const cache_create_ro_params& c_params,
    const cds::block3D_vectorno& c_alloc_blocks) {
  ER_DEBUG("Creating caches: alloc_blocks=[%s] (%zu)",
           rcppsw::to_string(c_alloc_blocks).c_str(),
           c_alloc_blocks.size());

  creation_result res;
  auto it = c_alloc_blocks.begin();
  for (auto& center : mc_centers) {
    auto dcenter = rmath::dvec2zvec(center, grid()->resolution().v());
    auto exists = std::find_if(c_params.current_caches.begin(),
                               c_params.current_caches.end(),
                               [&](const auto& c) {
                                 return dcenter == c->dcenter2D();
                               });
    /* static cache already exists */
    if (c_params.current_caches.end() != exists) {
      ER_DEBUG("Cache%d@%s/%s already exists",
               (*exists)->id().v(),
               rcppsw::to_string((*exists)->rcenter2D()).c_str(),
               rcppsw::to_string((*exists)->dcenter2D()).c_str());
      continue;
    }

    if (static_cast<uint>(std::distance(it, c_alloc_blocks.end())) <
        base_cache::kMinBlocks) {
      ER_WARN("Not enough blocks provided to construct cache@%s: %u < %zu",
              center.to_str().c_str(),
              static_cast<uint>(std::distance(it, c_alloc_blocks.end())),
              base_cache::kMinBlocks);
      continue;
    }
    auto it2 = it;
    std::advance(it, base_cache::kMinBlocks);
    cds::block3D_vectorno cache_i_blocks(it2, it);

    ER_INFO("Creating static cache@%s: blocks=[%s] (%zu)",
            center.to_str().c_str(),
            rcppsw::to_string(cache_i_blocks).c_str(),
            cache_i_blocks.size());
    res.created.push_back(create_single_cache(center,
                                              cache_i_blocks,
                                              c_params.t));
  } /* for(&center..) */

  return res;
} /* create_all() */

NS_END(depth1, support, fordyca);
