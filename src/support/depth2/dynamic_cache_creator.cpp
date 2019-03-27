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
#include "fordyca/ds/block_list.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/repr/arena_cache.hpp"
#include "fordyca/repr/base_block.hpp"
#include "fordyca/support/depth2/cache_center_calculator.hpp"
#include "fordyca/support/loop_utils/loop_utils.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);

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
    const ds::cache_vector& previous_caches,
    ds::block_vector& candidate_blocks,
    double cache_dim) {
  ds::cache_vector created_caches;

  ER_DEBUG("Creating caches: min_dist=%f,min_blocks=%u,free_blocks=[%s] (%zu)",
           m_min_dist,
           m_min_blocks,
           rcppsw::to_string(candidate_blocks).c_str(),
           candidate_blocks.size());

  ds::block_list used_blocks;
  for (size_t i = 0; i < candidate_blocks.size() - 1; ++i) {
    ds::block_list cache_i_blocks =
        cache_i_blocks_calc(used_blocks, candidate_blocks, i);

    /*
     * We now have all the blocks that are close enough to block i to be
     * included in a new cache, so attempt cache creation.
     */
    if (cache_i_blocks.size() >= m_min_blocks) {
      ds::cache_vector c_avoid =
          avoidance_caches_calc(previous_caches, created_caches);

      ds::block_list b_avoid =
          avoidance_blocks_calc(candidate_blocks, used_blocks, cache_i_blocks);

      if (auto center = cache_center_calculator(grid(), cache_dim)(
              cache_i_blocks, b_avoid, c_avoid)) {
        /*
         * We convert to discrete and then back to real coordinates so that our
         * cache's real location is always on an even multiple of the grid size,
         * which keeps asserts about cache extent from triggering right after
         * creation, which can happen otherwise.
         */
        auto cache_p = std::shared_ptr<repr::arena_cache>(
            create_single_cache(cache_i_blocks, rmath::uvec2dvec(center.get())));
        created_caches.push_back(cache_p);
      }

      /* Need to make sure we don't use these blocks in any other caches */
      used_blocks.insert(used_blocks.end(),
                         cache_i_blocks.begin(),
                         cache_i_blocks.end());
      ER_DEBUG("Used blocks=[%s]", rcppsw::to_string(used_blocks).c_str());
    }
  } /* for(i..) */

  ds::block_list free_blocks =
      avoidance_blocks_calc(candidate_blocks, used_blocks, ds::block_list());

  ER_ASSERT(creation_sanity_checks(created_caches, free_blocks),
            "One or more bad caches on creation");
  return created_caches;
} /* create_all() */

ds::cache_vector dynamic_cache_creator::avoidance_caches_calc(
    const ds::cache_vector& previous_caches,
    const ds::cache_vector& created_caches) const {
  ds::cache_vector avoid = previous_caches;
  avoid.insert(avoid.end(), created_caches.begin(), created_caches.end());
  return avoid;
} /* avoidance_caches_calc() */

ds::block_list dynamic_cache_creator::avoidance_blocks_calc(
    const ds::block_vector& candidate_blocks,
    const ds::block_list& used_blocks,
    const ds::block_list& cache_i_blocks) const {
  ds::block_list avoidance_blocks;
  std::copy_if(
      candidate_blocks.begin(),
      candidate_blocks.end(),
      std::back_inserter(avoidance_blocks),
      [&](const auto& b) __rcsw_pure {
        return used_blocks.end() ==
                   std::find(used_blocks.begin(), used_blocks.end(), b) &&
               cache_i_blocks.end() ==
                   std::find(cache_i_blocks.begin(), cache_i_blocks.end(), b);
      });
  return avoidance_blocks;
} /* avoidance_blocks_calc() */

ds::block_list dynamic_cache_creator::cache_i_blocks_calc(
    const ds::block_list& used_blocks,
    const ds::block_vector& candidates,
    uint index) const {
  ds::block_list src_blocks;

  /*
   * Block already in a new cache, so bail out.
   */
  if (std::find(used_blocks.begin(), used_blocks.end(), candidates[index]) !=
      used_blocks.end()) {
    return src_blocks;
  }
  /*
   * Add our anchor/target block to the list of blocks for the new cache. This
   * is OK to do even if there are no other blocks close enough to create a new
   * cache, because we have a minimum # blocks threshold that has to be met
   * anyway.
   */
  ER_TRACE("Add anchor block%d@%s to src list",
           candidates[index]->id(),
           candidates[index]->real_loc().to_str().c_str());
  src_blocks.push_back(candidates[index]);
  for (size_t i = index + 1; i < candidates.size(); ++i) {
    /*
     * If we find a block that is close enough to our anchor/target block, then
     * add to the src list.
     */
    if ((candidates[index]->real_loc() - candidates[i]->real_loc()).length() <=
        m_min_dist) {
      ER_ASSERT(std::find(src_blocks.begin(), src_blocks.end(), candidates[i]) ==
                    src_blocks.end(),
                "Block%d already on src list",
                candidates[i]->id());
      if (std::find(used_blocks.begin(), used_blocks.end(), candidates[i]) ==
          used_blocks.end()) {
        ER_TRACE("Add block %d@%s to src list",
                 candidates[i]->id(),
                 candidates[i]->real_loc().to_str().c_str());
        src_blocks.push_back(candidates[i]);
      }
    }
  } /* for(i..) */
  return src_blocks;
} /* cache_i_blocks_calc() */

NS_END(depth2, support, fordyca);
