/**
 * @file dynamic_cache_creator.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH2_DYNAMIC_CACHE_CREATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH2_DYNAMIC_CACHE_CREATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/base_cache_creator.hpp"
#include "fordyca/ds/block_cluster_vector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class dynamic_cache_creator
 * @ingroup fordyca support depth2
 *
 * @brief Handles creation of dynamic caches during simulation, given a set of
 * candidate blocks, and constraints on proximity, minimum # for a cache, etc.
 */
class dynamic_cache_creator : public base_cache_creator,
                              public rer::client<dynamic_cache_creator> {
 public:
  struct params {
    ds::arena_grid*      grid;
    rtypes::spatial_dist cache_dim;
    rtypes::spatial_dist min_dist;
    uint                 min_blocks;
  };
  explicit dynamic_cache_creator(const struct params* p);

  /**
   * @brief Create new caches in the arena from blocks that are close enough
   * together.
   */
  ds::cache_vector create_all(const ds::cache_vector& c_previous_caches,
                              const ds::block_cluster_vector& c_clusters,
                              const ds::block_vector& candidate_blocks,
                              rtypes::timestep t) override;

 private:
  /**
   * @brief Calculate the blocks to be used in the creation of a single new
   * cache.
   *
   * @param used_blocks The blocks that have been used to successfully create
   *                    other caches during this invocation of the creator.
   * @param candidates The total list of all blocks available for cache creation
   *                    when the creator was called.
   * @param index Our current index within the candidate vector.
   */
  ds::block_vector cache_i_blocks_alloc(const ds::block_vector& c_used_blocks,
                                        const ds::block_vector& candidates,
                                        uint index) const;

  /**
   *  @brief Blocks in this set:
   *
   * - Are not part of the set of blocks to be used for the cache we are
   *   currently attempting to create.
   * - Have not been already been made part of a cache earlier during this
   *   invocation of dynamic cache creation.
   *
   * Used for validation of cache creation.
   */
    ds::block_vector free_blocks_calc(const ds::block_vector& c_candidate_blocks,
                                      const ds::block_vector& c_used_blocks) const;

  /**
   * @brief Calculate the blocks a cache will absorb as a result of its center
   * beyond moved to deconflict with other caches/clusters/etc.
   *
   * @param candidate_blocks The total list of all blocks available for cache
   *                         creation when the creator was called.
   * @param cache_i_blocks   The blocks to be used in creating the new cache.
   * @param used_blocks      The blocks already used to create other caches.
   * @param center           The cache center.
   * @param cache_dim        The cache dimension.
   *
   * You *need* cache_i blocks, because if you omit them then you can have
   * blocks getting added to created caches twice, which causes all sorts of
   * problems. See #578.
   */
  ds::block_vector absorb_blocks_calc(
      const ds::block_vector& c_candidate_blocks,
      const ds::block_vector& c_cache_i_blocks,
      const ds::block_vector& c_used_blocks,
      const rmath::vector2u& c_center,
      rtypes::spatial_dist cache_dim) const;

  /**
   * @brief Create the set of caches that our new cache needs to avoid during
   * placement from the set of caches that existed prior to this invocation of
   * the creator + the set of caches we have created thus far during invocation.
   */
  ds::cache_vector avoidance_caches_calc(
      const ds::cache_vector& c_previous_caches,
      const ds::cache_vector& c_created_caches) const;

  /* clang-format off */
  const rtypes::spatial_dist mc_min_dist;
  const uint                 mc_min_blocks;
  /* clang-format on */
};


NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_DYNAMIC_CACHE_CREATOR_HPP_ */
