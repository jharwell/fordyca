/**
 * @file dynamic_cache_creator.cpp
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
#include "fordyca/support/depth2/dynamic_cache_creator.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/representation/arena_cache.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/dbg/dbg.hpp"
#include "fordyca/ds/block_list.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);

/*******************************************************************************
 * Class Constants
 ******************************************************************************/
const rmath::vector2i dynamic_cache_creator::kInvalidCacheCenter{-1, -1};

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dynamic_cache_creator::dynamic_cache_creator(ds::arena_grid* const grid,
                                             double cache_dim,
                                             double min_dist,
                                             uint min_blocks)
    : base_cache_creator(grid, cache_dim),
      ER_CLIENT_INIT("fordyca.support.depth2.dynamic_cache_creator"),
      m_min_dist(min_dist),
      m_min_blocks(min_blocks) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
ds::cache_vector dynamic_cache_creator::create_all(
    const ds::cache_vector& existing_caches,
    ds::block_vector& candidate_blocks,
    double cache_dim) {
  ds::cache_vector caches;

  ER_DEBUG("Creating caches: min_dist=%f,min_blocks=%u,free blocks=[%s] (%zu)",
           m_min_dist,
           m_min_blocks,
           dbg::blocks_list(candidate_blocks).c_str(),
           candidate_blocks.size());

  ds::block_list used_blocks;
  for (size_t i = 0; i < candidate_blocks.size() - 1; ++i) {
    ds::block_list cache_i_blocks = cache_blocks_calc(used_blocks,
                                                      candidate_blocks,
                                                      i);

    /*
     * We now have all the blocks that are close enough to block i to be
     * included in a new cache, so create the cache if we have enough.
     */
    if (cache_i_blocks.size() >= m_min_blocks) {
      /*
       * We need to also take the caches we have already created into account
       * when selecting the location of the new cache, not just the caches that
       * currently exist.
       */
      ds::cache_vector avoid = existing_caches;
      avoid.insert(avoid.end(), caches.begin(), caches.end());
      rmath::vector2i center = calc_center(cache_i_blocks, avoid, cache_dim);

      /*
       * We convert to discrete and then back to real coordinates so that our
       * cache's real location is always on an even multiple of the grid size,
       * which keeps asserts about cache extent from triggering right after
       * creation, which can happen otherwise.
       */
      if (kInvalidCacheCenter != center) {
        auto cache_p = std::shared_ptr<representation::arena_cache>(
            create_single_cache(cache_i_blocks, rmath::ivec2dvec(center)));
        caches.push_back(cache_p);
      }

      /* Need to make sure we don't use these blocks in any other caches */
      used_blocks.insert(used_blocks.end(),
                         cache_i_blocks.begin(),
                         cache_i_blocks.end());
      ER_DEBUG("Used blocks=[%s]", dbg::blocks_list(used_blocks).c_str());
    }
  } /* for(i..) */

  ER_ASSERT(creation_sanity_checks(caches),
            "One or more bad caches on creation");
  return caches;
} /* create_all() */

ds::block_list dynamic_cache_creator::cache_blocks_calc(
    const ds::block_list& used_blocks,
    const ds::block_vector& candidates,
    uint anchor_index) const {
  ds::block_list src_blocks;

  /*
   * Block already in a new cache, so bail out.
   */
  if (std::find(used_blocks.begin(),
                used_blocks.end(),
                candidates[anchor_index]) != used_blocks.end()) {
          return src_blocks;
  }
  /*
   * Add our anchor/target block to the list of blocks for the new cache. This
   * is OK to do even if there are no other blocks close enough to create a new
   * cache, because we have a minimum # blocks threshold that has to be met
   * anyway.
   */
  ER_TRACE("Add anchor block%d@%s to src list",
           candidates[anchor_index]->id(),
           candidates[anchor_index]->real_loc().to_str().c_str());
  src_blocks.push_back(candidates[anchor_index]);
  for (size_t i = anchor_index + 1; i < candidates.size(); ++i) {
    /*
     * If we find a block that is close enough to our anchor/target block, then
     * add to the src list.
     */
    if ((candidates[anchor_index]->real_loc() - candidates[i]->real_loc())
        .length() <= m_min_dist) {
      ER_ASSERT(std::find(src_blocks.begin(),
                          src_blocks.end(),
                          candidates[i]) == src_blocks.end(),
                "Block%d already on src list", candidates[i]->id());
      if (std::find(used_blocks.begin(),
                    used_blocks.end(),
                    candidates[i]) == used_blocks.end()) {
        ER_TRACE("Add block %d@%s to src list",
                 candidates[i]->id(),
                 candidates[i]->real_loc().to_str().c_str());
        src_blocks.push_back(candidates[i]);
      }
    }
  } /* for(i..) */
  return src_blocks;
} /* cache_blocks_calc() */


rmath::vector2i dynamic_cache_creator::calc_center(
    const ds::block_list& blocks,
    const ds::cache_vector& existing_caches,
    double cache_dim) const {
  double sumx = std::accumulate(
      std::begin(blocks), std::end(blocks), 0.0, [](double sum, const auto& b) {
        return sum + b->real_loc().x();
      });
  double sumy = std::accumulate(
      std::begin(blocks), std::end(blocks), 0.0, [](double sum, const auto& b) {
        return sum + b->real_loc().y();
      });

  rmath::vector2u center(sumx / blocks.size(), sumy / blocks.size());
  ER_DEBUG("Guess center=%s", center.to_str().c_str());

  /*
   * This needs to be done even if there are no other known caches, because the
   * guessed center might still be too close to the arena boundaries.
   */
  center = deconflict_arena_boundaries(cache_dim, center);

  /* If no existing caches, no possibility for conflict */
  if (existing_caches.empty()) {
    return rmath::vector2i(center.x(), center.y());
  }
  ER_DEBUG("Deconflict caches=[%s]", dbg::caches_list(existing_caches).c_str());

  /*
   * Every time we find an overlap we have to re-test all of the caches we've
   * already verified won't overlap with our new cache, because the move we
   * just made in x or y might have just caused an overlap.
   */
  uint i = 0;
  while (i++ < kOVERLAP_SEARCH_MAX_TRIES) {
    bool conflict = false;
    for (size_t j = 0; j < existing_caches.size(); ++j) {
      center = deconflict_arena_boundaries(cache_dim, center);
      auto pair = deconflict_existing_cache(*existing_caches[j], center);
      if (pair.first) {
        conflict = true;
        center = pair.second;
      }
    } /* for(j..) */
    if (!conflict) {
      break;
    }
  } /* for(i..) */

  /*
   * We have a set # of tries to fiddle with the new cache center, and if we
   * can't find anything conflict free in that many, bail out.
   */
  if (i >= kOVERLAP_SEARCH_MAX_TRIES) {
    ER_WARN("No conflict-free center found in %u tries: caches=[%s],blocks=[%s]",
            kOVERLAP_SEARCH_MAX_TRIES,
            dbg::caches_list(existing_caches).c_str(),
            dbg::blocks_list(blocks).c_str());
    return kInvalidCacheCenter;
  }

  return rmath::vector2i(center.x(), center.y());
} /* calc_center() */

NS_END(depth2, support, fordyca);
