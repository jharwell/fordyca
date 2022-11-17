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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/math/rng.hpp"

#include "cosm/foraging/ds/block_cluster_vector.hpp"
#include "cosm/ds/block3D_ht.hpp"

#include "fordyca/argos/support/caches/base_creator.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena {
class caching_arena_map;
} /* namespace cosm::arena */

NS_START(fordyca, argos, support, d2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dynamic_cache_creator
 * \ingroup argos support d2
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
class dynamic_cache_creator : public fascaches::base_creator,
                              public rer::client<dynamic_cache_creator> {
 public:
  struct params {
    /* clang-format off */
    carena::caching_arena_map* map;
    rspatial::euclidean_dist       cache_dim;
    rspatial::euclidean_dist       min_dist;
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
   *
   *
   * \param usable_blocks Those that been used (duh) to successfully create
   * caches this timestep. Once a block has been used during attempted cache
   * creation (even if creation failes), it is no longer available for the
   * creation of additional caches this timestep.
   *
   * This is because if creation fails and the cache is deleted, we STILL need
   * to add the blocks to the used list, because they have now been distributed
   * in the arena, possibly as member of a block cluster (for distributions
   * other than random), and are no longer free. See FORDYCA#673.
   *
   * \param absorbable_blocks A superset of \p usable_blocks which also contains
   * all free blocks in the arena (including those in clusters).
   */
  creation_result create_all(const fascaches::create_ro_params& c_params,
                             cds::block3D_vectorno&& usable_blocks,
                             cds::block3D_htno&& absorbable_blocks);

 private:
  struct cache_i_result {
    bool status{false};
    std::shared_ptr<carepr::arena_cache> cache{nullptr};
    cds::block3D_vectorno used{};
  };

  /**
   * \brief Calculate the blocks to be used in the creation of a single new
   * cache.
   *
   * \param c_usable_blocks The total list of all blocks available for cache
   *                        creation when the creator was called.
   * \param anchor The current anchor block.
   */
  cds::block3D_vectorno cache_i_blocks_alloc(
      const cds::block3D_vectorno& c_usable_blocks,
      cds::block3D_vectorno::iterator anchor) const;

  /**
   * \brief Calculate the blocks a cache will absorb as a result of its center
   * beyond moved to deconflict with other caches/clusters/etc.
   *
   * \param absorbable_blocks Free blocks which have not yet been successfully
   *                          used as part of cache creation.
   * \param cache_i_blocks   The blocks to be used in creating the new cache.
   * \param center           The cache center.
   * \param cache_dim        The cache dimension.
   *
   * You *need* cache_i_blocks, because if you omit them then you can have
   * blocks getting added to created caches twice, which causes all sorts of
   * problems. See FORDYCA#578.
   */
  cds::block3D_htno cache_i_alloc_from_absorbable(
      const cds::block3D_htno& c_absorbable_blocks,
      const cds::block3D_vectorno& c_cache_i_blocks,
      const rmath::vector2d& c_center,
      const rspatial::euclidean_dist& c_cache_dim) const;

  /**
   * \brief Create the set of caches that our new cache needs to avoid during
   * placement from the set of caches that existed prior to this invocation of
   * the creator + the set of caches we have created thus far during invocation.
   */
  cads::acache_vectorno avoidance_caches_calc(
      const cads::acache_vectorno& c_previous_caches,
      const cads::acache_vectoro& c_created_caches) const;


  /**
   * \brief Do the actual creation of a cache, once blocks have been allocated
   * for it.
   *
   * \return \c TRUE if creation was successful, and \c FALSE otherwise.
   */
  cache_i_result cache_i_create(const fascaches::create_ro_params& c_params,
                                const cds::block3D_vectorno& c_alloc_blocks,
                                const cds::block3D_htno& c_absorbable_blocks,
                                cads::acache_vectoro* created_caches);
  /**
   * \brief If a newly created cache failed verification checks, delete it.
   *
   * 1. Clear host cell.
   * 2. Redistribute the blocks that have been deposited in the host cell.
   *
   * Then the actual cache can safely be deleted.
   */
  void cache_delete(const cache_i_result& cache_i);

  /* clang-format off */
  const uint                 mc_min_blocks;
  const rspatial::euclidean_dist mc_min_dist;
  const bool                 mc_strict_constraints;

  rmath::rng*                m_rng;
  carena::caching_arena_map* m_map;
  /* clang-format on */
};


NS_END(d2, support, argos, fordyca);

