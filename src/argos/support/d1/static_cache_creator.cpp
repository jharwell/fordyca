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
#include "fordyca/argos/support/d1/static_cache_creator.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/repr/arena_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, d1);
using carepr::base_cache;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
static_cache_creator::static_cache_creator(
    carena::caching_arena_map* const map,
    const std::vector<rmath::vector2d>& centers,
    const rtypes::spatial_dist& cache_dim)
    : base_creator(map, cache_dim),
      ER_CLIENT_INIT("fordyca.support.d1.static_cache_creator"),
      mc_centers(centers) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
static_cache_creator::creation_result
static_cache_creator::create_all(const fascaches::create_ro_params& c_params,
                                 ds::block_alloc_map&& c_alloc_map,
                                 bool pre_dist) {
  creation_result res;
  for (auto& alloc_i : c_alloc_map) {
    ER_DEBUG("Cache%d alloced blocks: [%s] (%zu)",
             alloc_i.first,
             rcppsw::to_string(alloc_i.second).c_str(),
             mc_centers.size());
    auto rcenter = mc_centers[alloc_i.first];
    auto dcenter = rmath::dvec2zvec(rcenter, map()->grid_resolution().v());
    auto exists =
        std::find_if(c_params.current_caches.begin(),
                     c_params.current_caches.end(),
                     [&](const auto& c) { return dcenter == c->dcenter2D(); });
    /* static cache already exists */
    if (c_params.current_caches.end() != exists) {
      ER_DEBUG("Cache%d@%s/%s already exists",
               (*exists)->id().v(),
               rcppsw::to_string((*exists)->rcenter2D()).c_str(),
               rcppsw::to_string((*exists)->dcenter2D()).c_str());
      continue;
    }

    if (alloc_i.second.size() < base_cache::kMinBlocks) {
      ER_WARN("Not enough blocks provided to construct cache%d@%s/%s: %zu < %zu",
              alloc_i.first,
              rcppsw::to_string(rcenter).c_str(),
              rcppsw::to_string(dcenter).c_str(),
              alloc_i.second.size(),
              base_cache::kMinBlocks);
      continue;
    }

    ER_INFO("Creating static cache%d@%s/%s: blocks=[%s] (%zu)",
            alloc_i.first,
            rcppsw::to_string(rcenter).c_str(),
            rcppsw::to_string(dcenter).c_str(),
            rcppsw::to_string(alloc_i.second).c_str(),
            alloc_i.second.size());
    auto cache = create_single_cache(
        rcenter, std::move(alloc_i.second), c_params.t, pre_dist);
    res.created.push_back(std::move(cache));
  } /* for(&alloc_i..) */

  return res;
} /* create_all() */

NS_END(d1, support, argos, fordyca);
