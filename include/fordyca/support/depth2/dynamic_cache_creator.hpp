/**
 * \file dynamic_cache_creator.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "cosm/foraging/ds/block_cluster_vector.hpp"
#include "rcppsw/math/rng.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena {
class caching_arena_map;
} /* namespace cosm::arena */

NS_START(fordyca, support, depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dynamic_cache_creator
 * \ingroup support depth2
 *
 * \brief Handles creation of dynamic caches during simulation, given a set of
 * candidate blocks, and constraints on proximity, minimum # for a cache, etc.
 *
 * Cache creation is "best-effort", meaning that it attempts to create a
 * conflict free cache from the allocated blocks, but that is not always
 * possible, especially given the imprecise and stochastic nature of swarm
 * simulations. Verification of validity is only possible AFTER creation, so if
 * a newly created cache is found to be invalid, it is discarded and the process
 * of its creation reversed.
 */
class dynamic_cache_creator : public base_cache_creator,
                              public rer::client<dynamic_cache_creator> {
 public:
  struct params {
    /* clang-format off */
    carena::caching_arena_map* map;
    rtypes::spatial_dist       cache_dim;
    rtypes::spatial_dist       min_dist;
    uint                       min_blocks;
    bool                       strict_constraints;
    /* clang-format on */
  };
  dynamic_cache_creator(const params* p, rmath::rng* rng);
  dynamic_cache_creator(const dynamic_cache_creator&) = delete;
  dynamic_cache_creator& operator=(const dynamic_cache_creator&) = delete;

  /**
   * \brief Create new caches in the arena from blocks that are close enough
   * together.
   */
  cads::acache_vectoro create_all(
      const cache_create_ro_params& c_params,
      const cds::block3D_vectorno& c_alloc_blocks) override;

 private:
  /**
   * \brief Do the actual creation of a cache, once blocks have been allocated
   * for it.
   *
   * \return \c TRUE if creation was successful, and \c FALSE otherwise.
   */
  bool cache_i_create(const cache_create_ro_params& c_params,
                      const cds::block3D_vectorno& c_alloc_blocks,
                      cds::block3D_vectorno&& cache_i_blocks,
                      cads::acache_vectoro* created_caches,
                      cds::block3D_vectorno* used_blocks);

  /**
   * \brief If a newly created cache failed verification checks, delete it.
   *
   *
   * 1. Clear host cell.
   * 2. Redistribute the blocks that have been deposited in the host cell.
   *
   * Then the actual cache can safely be deleted.
   */
  void cache_delete(const cds::block3D_vectorno& cache_i_blocks,
                    cads::acache_vectoro* created_caches);
  /**
   * \brief Calculate the blocks to be used in the creation of a single new
   * cache.
   *
   * \param used_blocks The blocks that have been used to successfully create
   *                    other caches during this invocation of the creator.
   * \param alloc_blocks The total list of all blocks available for cache
   *                     creation when the creator was called.
   * \param index Our current index within the candidate vector.
   */
  cds::block3D_vectorno cache_i_blocks_alloc(
      const cds::block3D_vectorno& c_used_blocks,
      const cds::block3D_vectorno& c_alloc_blocks,
      uint index) const;

  /**
   * \brief Calculate the blocks a cache will absorb as a result of its center
   * beyond moved to deconflict with other caches/clusters/etc.
   *
   * \param alloc_blocks     The total list of all blocks available for cache
   *                         creation when the creator was called.
   * \param cache_i_blocks   The blocks to be used in creating the new cache.
   * \param used_blocks      The blocks already used to create other caches.
   * \param center           The cache center.
   * \param cache_dim        The cache dimension.
   *
   * You *need* cache_i blocks, because if you omit them then you can have
   * blocks getting added to created caches twice, which causes all sorts of
   * problems. See FORDYCA#578.
   */
  cds::block3D_vectorno absorb_blocks_calc(
      const cds::block3D_vectorno& c_alloc_blocks,
      const cds::block3D_vectorno& c_cache_i_blocks,
      const cds::block3D_vectorno& c_used_blocks,
      const rmath::vector2z& c_center,
      rtypes::spatial_dist cache_dim) const;

  /**
   * \brief Create the set of caches that our new cache needs to avoid during
   * placement from the set of caches that existed prior to this invocation of
   * the creator + the set of caches we have created thus far during invocation.
   */
  cads::acache_vectorno avoidance_caches_calc(
      const cads::acache_vectorno& c_previous_caches,
      const cads::acache_vectoro& c_created_caches) const;

  /* clang-format off */
  const bool                 mc_strict_constraints;
  const uint                 mc_min_blocks;
  const rtypes::spatial_dist mc_min_dist;

  rmath::rng*                m_rng;
  carena::caching_arena_map* m_map;
  /* clang-format on */
};


NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_DYNAMIC_CACHE_CREATOR_HPP_ */
