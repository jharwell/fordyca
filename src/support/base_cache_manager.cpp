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

#include "rcppsw/algorithm/transform.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<base_cache_manager::creation_blocks>
base_cache_manager::creation_blocks_alloc(
    const cds::block3D_vectorno& all_blocks,
    const cads::acache_vectorno& existing_caches,
    const cfds::block3D_cluster_vectorro& clusters,
    const block_alloc_filter_type& usable_filter,
    const block_alloc_filter_type& absorbable_filter) {
  creation_blocks allocated;

  std::copy_if(all_blocks.begin(),
               all_blocks.end(),
               std::back_inserter(allocated.usable),
               [&](const auto*b) { return usable_filter(b,
                                                        existing_caches,
                                                        clusters); });

  auto absorbable_transform = [&](auto* b) { return std::make_pair(b->id(), b); };

  ralg::transform_if(all_blocks.begin(),
                     all_blocks.end(),
                     std::inserter(allocated.absorbable,
                                   allocated.absorbable.begin()),
                     [&](const auto*b) { return absorbable_filter(b,
                                                                  existing_caches,
                                                                  clusters); },
                     absorbable_transform);

  if (creation_blocks_alloc_check(allocated, existing_caches)) {
    return boost::make_optional(allocated);
  } else {
    ER_FATAL_SENTINEL("Bad creation blocks allocation");
    return boost::none;
  }
} /* creation_blocks_alloc() */

bool base_cache_manager::creation_blocks_alloc_check(
    const creation_blocks& c_allocated,
    const cads::acache_vectorno& c_existing_caches) const {
  if (c_allocated.usable.size() < mc_config.dynamic.min_blocks) {
    /*
     * \todo Cannot use std::accumulate for these, because that doesn't work
     * with C++14/gcc7 when you are accumulating into a different type
     * (e.g. from a set of blocks into an int).
     */
    uint count = 0;
    std::for_each(
        c_allocated.usable.begin(), c_allocated.usable.end(), [&](const auto& b) {
          count +=
              (b->is_out_of_sight() ||
               std::any_of(c_existing_caches.begin(),
                           c_existing_caches.end(),
                           [&](const auto& c) { return !c->contains_block(b); }));
        });

    std::string accum;
    std::for_each(
        c_allocated.usable.begin(), c_allocated.usable.end(), [&](const auto& b) {
          accum += "b" + rcppsw::to_string(b->id()) + "->fb" +
                   rcppsw::to_string(b->md()->robot_id()) + ",";
        });
    ER_DEBUG("Block carry statuses: [%s]", accum.c_str());

    accum = "";
    std::for_each(
        c_allocated.usable.begin(), c_allocated.usable.end(), [&](const auto& b) {
          accum += "b" + rcppsw::to_string(b->id()) + "->" +
                   b->ranchor2D().to_str() + "/" + b->danchor2D().to_str() + ",";
        });
    ER_DEBUG("Block locations: [%s]", accum.c_str());

    ER_CHECK(c_allocated.usable.size() - count < mc_config.dynamic.min_blocks,
             "For new caches, %zu blocks SHOULD be available, but only %zu "
             "are (min=%u)",
             c_allocated.usable.size() - count,
             c_allocated.usable.size(),
             mc_config.dynamic.min_blocks);
  }
  if (c_allocated.usable.size() < mc_config.static_.size) {
    ER_WARN("Free block count < min blocks for new caches (%zu < %u)",
            c_allocated.usable.size(),
            mc_config.dynamic.min_blocks);
  }
  return true;

error:
  return false;
} /* creation_blocks_alloc_check() */

void base_cache_manager::bloctree_update(const cads::acache_vectoro& caches) {
  cads::acache_vectorro created;
  for (const auto& c : caches) {
    created.push_back(c.get());
  } /* for(&b..) */
  m_map->created_caches(created);

  for (const auto& cache : caches) {
    for (auto* block : cache->blocks()) {
      m_map->bloctree_update(block, carena::locking::ekALL_HELD);
    } /* for(*block..) */
  } /* for(&cache..) */

  m_map->created_caches_clear();
} /* bloctree_update() */

NS_END(support, fordyca);
