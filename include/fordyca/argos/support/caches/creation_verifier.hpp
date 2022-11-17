/**
 * \file creation_verifier.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "rcppsw/er/client.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"

#include "cosm/arena/ds/nest_vector.hpp"
#include "cosm/ds/block3D_vector.hpp"
#include "cosm/ds/block3D_ht.hpp"
#include "cosm/arena/ds/cache_vector.hpp"
#include "cosm/foraging/ds/block_cluster_vector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena {
class caching_arena_map;
} /* namespace cosm::arena */

NS_START(fordyca, argos, support, caches);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class creation_verifier
 * \ingroup argos support caches
 *
 *
 * \brief Provides verification of newly created caches, indicating to calling
 *        classes if the newly created cache should be kept or discarded
 *        (according to configuration).
 */
class creation_verifier : public rer::client<creation_verifier> {
 public:
  creation_verifier(carena::caching_arena_map* map,
                     const rspatial::euclidean_dist& cache_dim,
                     bool strict_constraints);

  creation_verifier(const creation_verifier&) = delete;
  creation_verifier& operator=(const creation_verifier&) = delete;

  /**
   * \brief Is the created cache OK, or should it be discarded?
   */
  bool verify_single(const carepr::arena_cache* cache,
                     const cads::acache_vectorro& c_caches,
                     const cds::block3D_vectorno& c_all_blocks,
                     const cfds::block3D_cluster_vectorro& c_clusters) const;

  /**
   * \brief Basic sanity checks on a set of newly created caches.
   *
   * - No block contained in one cache is contained in another.
   * - No two newly created caches overlap.
   * - No block that is not currently contained in a cache overlaps any cache.
   * - No cache overlaps a block cluster.
   *
   * \param c_caches The created caches.
   * \param c_free_blocks Blocks that are not carried by a robot or part of a
   *                     newly created cache.
   * \param c_clusters Current block clusters in the arena.
   * \param c_nests The nests in the arena.
   *
   * \return \c TRUE iff no errors/inconsistencies are found, \c FALSE
   * otherwise. If \c FALSE, then the problem MUST be with the most newly
   * created cache, since all previously created caches already have passed all
   * checks.
   */
  bool sanity_checks(const cads::acache_vectorro& c_caches,
                     const cds::block3D_vectorno& c_free_blocks,
                     const cfds::block3D_cluster_vectorro& c_clusters,
                     const cads::nest_vectorro& c_nests) const RCPPSW_PURE;

 private:
  bool sanity_check_internal_consistency(const carepr::arena_cache* cache) const
      RCPPSW_PURE;
  bool sanity_check_cross_consistency(
      const cads::acache_vectorro& c_caches) const RCPPSW_PURE;
  bool sanity_check_cache_overlap(const cads::acache_vectorro& c_caches) const;
  bool
  sanity_check_free_block_overlap(const carepr::arena_cache* cache,
                                  const cds::block3D_vectorno& free_blocks) const;
  bool sanity_check_block_cluster_overlap(
      const carepr::arena_cache* cache,
      const cfds::block3D_cluster_vectorro& clusters) const;

  bool sanity_check_nest_overlap(const carepr::arena_cache* cache,
                                 const cads::nest_vectorro& nests) const;

  /* clang-format off */
  const rspatial::euclidean_dist mc_cache_dim;
  const bool                 mc_strict_constraints;

  carena::caching_arena_map* m_map;
  /* clang-format on */
};
NS_END(caches, support, argos, fordyca);

