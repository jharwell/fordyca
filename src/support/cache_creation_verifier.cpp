/**
 * \file cache_creation_verifier.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/cache_creation_verifier.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/repr/nest.hpp"
#include "cosm/arena/free_blocks_calculator.hpp"
#include "cosm/foraging/repr/block_cluster.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);
using cds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_creation_verifier::cache_creation_verifier(carena::caching_arena_map* const map,
                                                 const rtypes::spatial_dist& cache_dim,
                                                 bool strict_constraints)
    : ER_CLIENT_INIT("fordyca.support.cache_creation_verifier"),
      mc_cache_dim(cache_dim),
      mc_strict_constraints(strict_constraints),
      m_map(map) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

bool cache_creation_verifier::verify_single(
    RCPPSW_UNUSED const carepr::arena_cache* cache,
    const cads::acache_vectorro& c_caches,
    const cds::block3D_vectorno& c_all_blocks,
    const cfds::block3D_cluster_vectorro& c_clusters) const {
  auto free_blocks =
      carena::free_blocks_calculator(false)(c_all_blocks, c_caches);

  bool sanity_ok = sanity_checks(c_caches,
                                 free_blocks,
                                 c_clusters,
                                 m_map->nests());
  if (!sanity_ok) {
    if (mc_strict_constraints) {
      ER_WARN("Bad cache%d@%s/%s creation--discard (strict constraints)",
              cache->id().v(),
              rcppsw::to_string(cache->rcenter2D()).c_str(),
              rcppsw::to_string(cache->dcenter2D()).c_str());
      return false;
    } else {
      ER_WARN("Bad cache%d@%s/%s creation--keep (loose constraints)",
              cache->id().v(),
              rcppsw::to_string(cache->rcenter2D()).c_str(),
              rcppsw::to_string(cache->dcenter2D()).c_str());
      return true;
    }
  } else {
    ER_INFO("Cache%d@%s/%s creation sanity checks OK",
            cache->id().v(),
            rcppsw::to_string(cache->rcenter2D()).c_str(),
            rcppsw::to_string(cache->dcenter2D()).c_str());
    return true;
  }
} /* verify_single() */

bool cache_creation_verifier::sanity_checks(
    const cads::acache_vectorro& c_caches,
    const cds::block3D_vectorno& c_free_blocks,
    const cfds::block3D_cluster_vectorro& c_clusters,
    const cads::nest_vectorro& c_nests) const {
  for (const auto& c : c_caches) {
    ER_CHECK(sanity_check_internal_consistency(c),
             "Cache%d@%s/%s not internally consistent",
             c->id().v(),
             rcppsw::to_string(c->rcenter2D()).c_str(),
             rcppsw::to_string(c->dcenter2D()).c_str());
  }
  for (const auto& c : c_caches) {
    ER_CHECK(sanity_check_free_block_overlap(c, c_free_blocks),
             "One or more caches overlap with free blocks");
  } /* for(&c..) */

  if (!mc_strict_constraints) {
    for (const auto& c : c_caches) {
      ER_CHECK(sanity_check_block_cluster_overlap(c, c_clusters),
               "One or more caches overlap with one or more block clusters");
    } /* for(&c..) */
  }

  for (const auto& c : c_caches) {
    ER_CHECK(sanity_check_nest_overlap(c, c_nests),
             "One or more caches overlap with one or more nests");
  } /* for(&c..) */

  ER_CHECK(sanity_check_cross_consistency(c_caches),
           "Two or more caches not cross-consistent");

  ER_CHECK(sanity_check_cache_overlap(c_caches), "Two or more caches overlap");

  return true;

error:
  return false;
} /* sanity_checks() */

bool cache_creation_verifier::sanity_check_internal_consistency(
    const carepr::arena_cache* cache) const {
  auto& cell = m_map->access<arena_grid::kCell>(cache->dcenter2D());
  ER_CHECK(cell.fsm().state_has_cache(),
           "Cell@%s not in HAS_CACHE state",
           cell.loc().to_str().c_str());
  ER_CHECK(nullptr != cell.cache(),
           "Cell@%s in HAS_CACHE state but does not contain cache",
           cell.loc().to_str().c_str());
  ER_CHECK(cache->n_blocks() == cell.block_count(),
           "Cache/cell disagree on # of blocks: cache=%zu/cell=%zu",
           cache->n_blocks(),
           cell.block_count());
  ER_CHECK(cache->n_blocks() >= carepr::base_cache::kMinBlocks,
           "Cache does not have enough blocks: %zu < %zu",
           cache->n_blocks(),
           carepr::base_cache::kMinBlocks);
  for (const auto* b : cache->blocks()) {
    ER_CHECK(b->danchor2D() == cache->dcenter2D(),
             "Block%d@%s not in cache host cell@%s",
             b->id().v(),
             rcppsw::to_string(b->danchor2D()).c_str(),
             rcppsw::to_string(cache->dcenter2D()).c_str())
  } /* for(*b..) */

  return true;

error:
  return false;
} /* sanity_check_internal_consistency() */

bool cache_creation_verifier::sanity_check_free_block_overlap(
    const carepr::arena_cache* cache,
    const cds::block3D_vectorno& free_blocks) const {
  auto xspan = cache->xrspan();
  auto yspan = cache->yrspan();

  for (const auto& b : free_blocks) {
    ER_CHECK(!(xspan.overlaps_with(b->xrspan()) &&
               yspan.overlaps_with(b->yrspan())),
             "Cache%d xspan=%s,yspan=%s overlaps block%d "
             "xspan=%s,yspan=%s",
             cache->id().v(),
             rcppsw::to_string(xspan).c_str(),
             rcppsw::to_string(yspan).c_str(),
             b->id().v(),
             rcppsw::to_string(b->xrspan()).c_str(),
             rcppsw::to_string(b->yrspan()).c_str());
  } /* for(&b..) */

  return true;

error:
  return false;
} /* sanity_check_free_block_overlap() */

bool cache_creation_verifier::sanity_check_block_cluster_overlap(
    const carepr::arena_cache* cache,
    const cfds::block3D_cluster_vectorro& clusters) const {
  for (const auto& cluster : clusters) {
    ER_CHECK(!(cache->xrspan().overlaps_with(cluster->xrspan()) &&
               cache->yrspan().overlaps_with(cluster->yrspan())),
             "Cache%d xspan=%s,yspan=%s overlaps cluster w/xspan=%s,yspan=%s",
             cache->id().v(),
             rcppsw::to_string(cache->xrspan()).c_str(),
             rcppsw::to_string(cache->yrspan()).c_str(),
             rcppsw::to_string(cluster->xrspan()).c_str(),
             rcppsw::to_string(cluster->yrspan()).c_str());
  } /* for(&cluster..) */
  return true;

error:
  return false;
} /* sanity_check_block_cluster_overlap() */

bool cache_creation_verifier::sanity_check_nest_overlap(
    const carepr::arena_cache* cache,
    const cads::nest_vectorro& nests) const {
  for (const auto& nest : nests) {
    ER_CHECK(!(cache->xrspan().overlaps_with(nest->xrspan()) &&
               cache->yrspan().overlaps_with(nest->yrspan())),
             "Cache%d xspan=%s,yspan=%s overlaps nest%d w/xspan=%s,yspan=%s",
             cache->id().v(),
             rcppsw::to_string(cache->xrspan()).c_str(),
             rcppsw::to_string(cache->yrspan()).c_str(),
             nest->id().v(),
             rcppsw::to_string(nest->xrspan()).c_str(),
             rcppsw::to_string(nest->yrspan()).c_str());
  } /* for(&nest..) */
  return true;

error:
  return false;
} /* sanity_check_nest_overlap() */

bool cache_creation_verifier::sanity_check_cross_consistency(
    const cads::acache_vectorro& caches) const {
  for (const auto& c1 : caches) {
    auto c1_xspan = c1->xrspan();
    auto c1_yspan = c1->yrspan();

    /* check caches contain different blocks and no duplicates */
    for (const auto& c2 : caches) {
      if (c1->id() == c2->id()) {
        continue;
      }
      for (const auto* b : c1->blocks()) {
        ER_CHECK(c1->blocks().end() ==
                     std::adjacent_find(c1->blocks().begin(), c1->blocks().end()),
                 "Multiple blocks with the same ID in cache%d",
                 c1->id().v());

        ER_CHECK(!c2->contains_block(b),
                 "Block%d contained in both cache%d and cache%d",
                 b->id().v(),
                 c1->id().v(),
                 c2->id().v());
      } /* for(&b..) */
    } /* for(&c2..) */
  } /* for(&c1..) */
  return true;

error:
  return false;
} /* sanity_check_cross_consistency() */

bool cache_creation_verifier::sanity_check_cache_overlap(
    const cads::acache_vectorro& caches) const {
  for (const auto& c1 : caches) {
    auto c1_xspan = c1->xrspan();
    auto c1_yspan = c1->yrspan();

    /* Check caches do not overlap */
    for (const auto& c2 : caches) {
      if (c1->id() == c2->id()) {
        continue;
      }
      auto c2_xspan = c2->xrspan();
      auto c2_yspan = c2->yrspan();

      bool overlap = c1_xspan.overlaps_with(c2_xspan) &&
                     c1_yspan.overlaps_with(c2_yspan);
      ER_CHECK(!overlap,
               "Cache%d@%s/%s xspan=%s, yspan=%s overlaps cache%d@%s/%s "
               "xspan=%s,yspan=%s",
               c1->id().v(),
               rcppsw::to_string(c1->rcenter2D()).c_str(),
               rcppsw::to_string(c1->dcenter2D()).c_str(),
               rcppsw::to_string(c1_xspan).c_str(),
               rcppsw::to_string(c1_yspan).c_str(),
               c2->id().v(),
               rcppsw::to_string(c2->rcenter2D()).c_str(),
               rcppsw::to_string(c2->dcenter2D()).c_str(),
               rcppsw::to_string(c2_xspan).c_str(),
               rcppsw::to_string(c2_yspan).c_str());
    } /* for(&c2..) */
  } /* for(c1...) */

  return true;

error:
  return false;
} /* sanity_check_cache_overlap() */

NS_END(support, fordyca);
