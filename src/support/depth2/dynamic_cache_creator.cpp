/**
 * \file dynamic_cache_creator.cpp
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
#include "fordyca/support/depth2/dynamic_cache_creator.hpp"

#include "cosm/repr/base_block2D.hpp"

#include "fordyca/ds/block_list.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/repr/arena_cache.hpp"
#include "fordyca/support/depth2/cache_center_calculator.hpp"
#include "fordyca/support/utils/loop_utils.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dynamic_cache_creator::dynamic_cache_creator(const params* const p,
                                             rmath::rng* rng)
    : base_cache_creator(p->grid, p->cache_dim),
      ER_CLIENT_INIT("fordyca.support.depth2.dynamic_cache_creator"),
      mc_min_dist(p->min_dist),
      mc_min_blocks(p->min_blocks),
      m_rng(rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
ds::cache_vector dynamic_cache_creator::create_all(
    const cache_create_ro_params& c_params,
    const ds::block_vector& c_alloc_blocks) {
  ds::cache_vector created_caches;

  ER_DEBUG("Creating caches: min_dist=%f,min_blocks=%u,free_blocks=[%s] (%zu)",
           mc_min_dist.v(),
           mc_min_blocks,
           rcppsw::to_string(c_alloc_blocks).c_str(),
           c_alloc_blocks.size());

  ds::block_vector used_blocks;
  for (size_t i = 0; i < c_alloc_blocks.size() - 1; ++i) {
    ds::block_vector cache_i_blocks =
        cache_i_blocks_alloc(used_blocks, c_alloc_blocks, i);

    /*
     * We now have all the blocks that are close enough to block i to be
     * included in a new cache, so attempt cache creation.
     */
    if (cache_i_blocks.size() >= mc_min_blocks) {
      ds::cache_vector c_avoid =
          avoidance_caches_calc(c_params.current_caches, created_caches);

      if (auto center = cache_center_calculator(grid(), cache_dim())(
              cache_i_blocks, c_avoid, c_params.clusters, m_rng)) {
        /*
         * If we found a conflict free cache center, we need to check if there
         * are free blocks that are now within the extent of the cache to be due
         * to the center moving. If so, we absorb them into the block list for
         * the new cache.
         */
        auto absorb_blocks = absorb_blocks_calc(
            c_alloc_blocks, cache_i_blocks, used_blocks, *center, cache_dim());
        ER_DEBUG("Absorb blocks=[%s]", rcppsw::to_string(absorb_blocks).c_str());

        cache_i_blocks.insert(cache_i_blocks.end(),
                              absorb_blocks.begin(),
                              absorb_blocks.end());
        /*
         * We convert to discrete and then back to real coordinates so that our
         * cache's real location is always on an even multiple of the grid size,
         * which keeps asserts about cache extent from triggering right after
         * creation, which can happen otherwise.
         */
        auto cache_p = std::shared_ptr<repr::arena_cache>(create_single_cache(
            rmath::uvec2dvec(*center), cache_i_blocks, c_params.t));
        created_caches.push_back(cache_p);
      }

      /* Need to make sure we don't use these blocks in any other caches */
      used_blocks.insert(used_blocks.end(),
                         cache_i_blocks.begin(),
                         cache_i_blocks.end());
      ER_DEBUG("Used blocks=[%s]", rcppsw::to_string(used_blocks).c_str());
    }
  } /* for(i..) */

  ds::block_vector free_blocks =
      utils::free_blocks_calc(created_caches, c_alloc_blocks);

  ER_ASSERT(
      creation_sanity_checks(created_caches, free_blocks, c_params.clusters),
      "One or more bad caches on creation");
  return created_caches;
} /* create_all() */

ds::cache_vector dynamic_cache_creator::avoidance_caches_calc(
    const ds::cache_vector& c_previous_caches,
    const ds::cache_vector& c_created_caches) const {
  ds::cache_vector avoid = c_previous_caches;
  avoid.insert(avoid.end(), c_created_caches.begin(), c_created_caches.end());
  return avoid;
} /* avoidance_caches_calc() */

ds::block_vector dynamic_cache_creator::absorb_blocks_calc(
    const ds::block_vector& c_alloc_blocks,
    const ds::block_vector& c_cache_i_blocks,
    const ds::block_vector& c_used_blocks,
    const rmath::vector2u& c_center,
    rtypes::spatial_dist cache_dim) const {
  ds::block_vector absorb_blocks;
  std::copy_if(c_alloc_blocks.begin(),
               c_alloc_blocks.end(),
               std::back_inserter(absorb_blocks),
               [&](const auto& b) RCSW_PURE {
                 auto xspan = rmath::ranged(c_center.x() - cache_dim.v() / 2.0,
                                            c_center.x() + cache_dim.v() / 2.0);
                 auto yspan = rmath::ranged(c_center.y() - cache_dim.v() / 2.0,
                                            c_center.y() + cache_dim.v() / 2.0);
                 return c_used_blocks.end() == std::find(c_used_blocks.begin(),
                                                         c_used_blocks.end(),
                                                         b) &&
                        c_cache_i_blocks.end() ==
                            std::find(c_cache_i_blocks.begin(),
                                      c_cache_i_blocks.end(),
                                      b) &&
                        xspan.overlaps_with(b->xspan()) &&
                        yspan.overlaps_with(b->yspan());
               });
  return absorb_blocks;
} /* absorb_blocks_calc() */

ds::block_vector dynamic_cache_creator::cache_i_blocks_alloc(
    const ds::block_vector& c_used_blocks,
    const ds::block_vector& c_alloc_blocks,
    uint index) const {
  ds::block_vector src_blocks;

  /*
   * Block already in a new cache, so bail out.
   */
  if (std::find(c_used_blocks.begin(),
                c_used_blocks.end(),
                c_alloc_blocks[index]) != c_used_blocks.end()) {
    return src_blocks;
  }
  /*
   * Add our anchor/target block to the list of blocks for the new cache. This
   * is OK to do even if there are no other blocks close enough to create a new
   * cache, because we have a minimum # blocks threshold that has to be met
   * anyway.
   */
  ER_TRACE("Add anchor block%d@%s to src list",
           c_alloc_blocks[index]->id().v(),
           c_alloc_blocks[index]->rloc().to_str().c_str());
  src_blocks.push_back(c_alloc_blocks[index]);
  for (size_t i = index + 1; i < c_alloc_blocks.size(); ++i) {
    /*
     * If we find a block that is close enough to our anchor/target block, then
     * add to the src list.
     */
    if (mc_min_dist >=
        (c_alloc_blocks[index]->rloc() - c_alloc_blocks[i]->rloc()).length()) {
      ER_ASSERT(std::find(src_blocks.begin(),
                          src_blocks.end(),
                          c_alloc_blocks[i]) == src_blocks.end(),
                "Block%d already on src list",
                c_alloc_blocks[i]->id().v());
      if (std::find(c_used_blocks.begin(),
                    c_used_blocks.end(),
                    c_alloc_blocks[i]) == c_used_blocks.end()) {
        ER_TRACE("Add block %d@%s to src list",
                 c_alloc_blocks[i]->id().v(),
                 c_alloc_blocks[i]->rloc().to_str().c_str());
        src_blocks.push_back(c_alloc_blocks[i]);
      }
    }
  } /* for(i..) */
  return src_blocks;
} /* cache_i_blocks_alloc() */

NS_END(depth2, support, fordyca);
