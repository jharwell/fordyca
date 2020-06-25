/**
 * \file base_cache_creator.cpp
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
#include "fordyca/support/base_cache_creator.hpp"

#include "cosm/ds/operations/cell2D_cache_extent.hpp"
#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/arena/repr/light_type_index.hpp"
#include "cosm/foraging/repr/block_cluster.hpp"
#include "cosm/repr/base_block3D.hpp"

#include "fordyca/events/cell2D_empty.hpp"
#include "fordyca/support/utils/loop_utils.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);
using cds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_cache_creator::base_cache_creator(cds::arena_grid* const grid,
                                       rtypes::spatial_dist cache_dim)
    : ER_CLIENT_INIT("fordyca.support.base_cache_creator"),
      mc_cache_dim(cache_dim),
      m_grid(grid) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::unique_ptr<carepr::arena_cache> base_cache_creator::create_single_cache(
    const rmath::vector2z& center,
    cds::block3D_vectorno blocks,
    const rtypes::timestep& t) {
  ER_ASSERT(center.x() > 0 && center.y() > 0,
            "Center@%s is not positive definite",
            center.to_str().c_str());
  /*
   * The cell that will be the location of the new cache may already contain a
   * block. If so, it should be added to the list of blocks for the cache.
   */
  auto& cell = m_grid->access<arena_grid::kCell>(center);
  if (cell.state_has_block()) {
    ER_ASSERT(nullptr != cell.block3D(),
              "Cell@%s does not have block",
              cell.loc().to_str().c_str());
    ER_ASSERT(!cell.block3D()->is_out_of_sight(),
              "Allocated block%d is out of sight",
              cell.block3D()->id().v());

    /*
     * If the host cell for the cache already contains a block, we don't want to
     * unconditionally add said block to the list of blocks for the new cache,
     * because if the cache was created using some function of free blocks in
     * the arena, then that block could already be in the list, and we would be
     * double adding it, which will cause problems later (dynamic cache
     * creation). However, it may also NOT be in the list of blocks to use for
     * the new cache, in which case we need to add in (static cache creation).
     */
    if (blocks.end() == std::find(blocks.begin(), blocks.end(), cell.block3D())) {

      /*
       * We use insert() instead of push_back() here so that if there was a
       * leftover block on the cell where a cache used to be that is also where
       * this cache is being created, it becomes the "front" of the cache, and
       * will be the first block picked up by a robot from the new cache. This
       * helps to ensure fairness/better statistics for the simulations.
       */
      ER_DEBUG("Add block%d from cache host cell@%s to block vector",
               cell.block3D()->id().v(),
               rcppsw::to_string(cell.loc()).c_str());
      blocks.insert(blocks.begin(), cell.block3D());
    }
  }

  /*
   * We don't need to lock around the cell empty and block drop events because
   * cache creation always happens AFTER all robots have had their control steps
   * run, never DURING. We are not REALLY holding all the locks, but no need to
   * grab them in a non-concurrent context.
   */
  for (auto& block : blocks) {
    events::cell2D_empty_visitor op(block->danchor2D());
    op.visit(m_grid->access<arena_grid::kCell>(block->danchor2D()));
  } /* for(block..) */

  /*
   * Loop through all blocks and deposit them in the cache host cell, which
   * will be in the HAS_CACHE state after this loop, but will not yet have a
   * cache as its entity.
   */
  for (auto& block : blocks) {
    caops::free_block_drop_visitor op(block,
                                      center,
                                      m_grid->resolution(),
                                      carena::arena_map_locking::ekALL_HELD);
    op.visit(m_grid->access<arena_grid::kCell>(op.coord()));
  } /* for(block..) */
  ER_DEBUG("All %zu blocks now in host cell%s",
           blocks.size(),
           rcppsw::to_string(center).c_str());

  /* create the cache! */
  cds::block3D_vectorno block_vec(blocks.begin(), blocks.end());
  auto ret = std::make_unique<carepr::arena_cache>(
      carepr::arena_cache::params{mc_cache_dim,
            m_grid->resolution(),
            rmath::zvec2dvec(center, m_grid->resolution().v()),
            block_vec,
            rtypes::constants::kNoUUID},
      carepr::light_type_index()[carepr::light_type_index::kCache]);
  ret->creation_ts(t);
  ER_ASSERT(center == ret->dcenter2D(),
            "Created cache%d has bad center: %s != %s",
            ret->id().v(),
            rcppsw::to_string(ret->dcenter2D()).c_str(),
            rcppsw::to_string(center).c_str());
  ER_INFO("Created cache%d@%s/%s,xspan=%s/%s,yspan=%s/%s with %zu blocks [%s]",
          ret->id().v(),
          rcppsw::to_string(ret->rcenter2D()).c_str(),
          rcppsw::to_string(ret->dcenter2D()).c_str(),
          rcppsw::to_string(ret->xrspan()).c_str(),
          rcppsw::to_string(ret->xdspan()).c_str(),
          rcppsw::to_string(ret->yrspan()).c_str(),
          rcppsw::to_string(ret->ydspan()).c_str(),
          ret->n_blocks(),
          rcppsw::to_string(blocks).c_str());

  /* Update host cell */
  cell.entity(ret.get());

  return ret;
} /* create_single_cache() */

void base_cache_creator::configure_cache_extents(cads::acache_vectoro& caches) {
  /*
   * To set the cells covered by a cache's extent, we simply send them a
   * CACHE_EXTENT event, EXCEPT for the cell that hosts the actual cache,
   * because it is currently in the HAS_CACHE state as part of the cache
   * creation process. For that cell, we just set its entity, as the cell FSM is
   * already in the correct state.
   */
  for (auto& cache : caches) {
    auto xspan = cache->xdspan();
    auto yspan = cache->ydspan();

    for (size_t i = xspan.lb(); i <= xspan.ub() ; ++i) {
      for (size_t j = yspan.lb(); j <= yspan.ub(); ++j) {
        auto dcoord = rmath::vector2z(i, j);
        auto rcoord = rmath::zvec2dvec(dcoord, m_grid->resolution().v());
        auto& cell = m_grid->access<arena_grid::kCell>(dcoord);

        ER_ASSERT(cache->contains_point2D(rcoord),
                  "Cache%d@%s/%s xspan=%s,yspan=%s does not contain %s",
                  cache->id().v(),
                  rcppsw::to_string(cache->rcenter2D()).c_str(),
                  rcppsw::to_string(cache->dcenter2D()).c_str(),
                  rcppsw::to_string(cache->xrspan()).c_str(),
                  rcppsw::to_string(cache->yrspan()).c_str(),
                  rcppsw::to_string(rcoord).c_str());

        if (dcoord != cache->dcenter2D()) {
          ER_ASSERT(!cell.state_in_cache_extent(),
                    "Cell@%s already in CACHE_EXTENT",
                    rcppsw::to_string(dcoord).c_str());
          ER_TRACE("Configure cache%d cell@%s: CACHE_EXTENT",
                   cache->id().v(),
                   rcppsw::to_string(dcoord).c_str());
          cdops::cell2D_cache_extent_visitor e(dcoord, cache.get());
          e.visit(cell);
        } else {
          ER_ASSERT(cell.state_has_cache(),
                    "Cache host cell@%s not in HAS_CACHE",
                    rcppsw::to_string(dcoord).c_str());
        }
      } /* for(j..) */
    }   /* for(i..) */
  }     /* for(cache..) */
} /* configure_cache_extents() */

bool base_cache_creator::creation_sanity_checks(
    const cads::acache_vectoro& caches,
    const cds::block3D_vectorno& free_blocks,
    const cfds::block3D_cluster_vector& clusters) const {
  ER_CHECK(sanity_check_internal_consistency(caches),
           "One or more caches not internally consistent");

  ER_CHECK(sanity_check_cross_consistency(caches),
           "Two or more caches not cross-consistent");

  ER_CHECK(sanity_check_cache_overlap(caches),
           "Two or more caches overlap");

  ER_CHECK(sanity_check_free_block_overlap(caches, free_blocks),
           "One or more caches overlap with free blocks");

  ER_CHECK(sanity_check_block_cluster_overlap(caches, clusters),
           "One or more caches overlap with one or more block clusters");

  return true;

error:
  return false;
} /* creation_sanity_checks() */

bool base_cache_creator::sanity_check_internal_consistency(
    const cads::acache_vectoro& caches) const {
  for (auto& c1 : caches) {
    auto& cell = m_grid->access<arena_grid::kCell>(c1->dcenter2D());
    ER_CHECK(cell.fsm().state_has_cache(),
             "Cell@%s not in HAS_CACHE state",
             cell.loc().to_str().c_str());
    ER_CHECK(nullptr != cell.cache(),
             "Cell@%s in HAS_CACHE state but does not contain cache",
             cell.loc().to_str().c_str());
    ER_CHECK(c1->n_blocks() == cell.block_count(),
             "Cache/cell disagree on # of blocks: cache=%zu/cell=%zu",
             c1->n_blocks(),
             cell.block_count());
  } /* for(c1..) */
  return true;

error:
  return false;
} /* sanity_check_internal_consistency() */

bool base_cache_creator::sanity_check_cross_consistency(
    const cads::acache_vectoro& caches) const {

  for (auto& c1 : caches) {
    auto c1_xspan = c1->xrspan();
    auto c1_yspan = c1->yrspan();

      /* check caches contain different blocks and no duplicates */
    for (auto& c2 : caches) {
      if (c1->id() == c2->id()) {
        continue;
      }
      for (auto& b : c1->blocks()) {
        ER_CHECK(c1->blocks().end() == std::adjacent_find(c1->blocks().begin(),
                                                          c1->blocks().end()),
                 "Multiple blocks with the same ID in cache%d",
                 c1->id().v());

        ER_CHECK(!c2->contains_block(b),
                 "Block%d contained in both cache%d and cache%d",
                 b->id().v(),
                 c1->id().v(),
                 c2->id().v());
      } /* for(&b..) */
    }   /* for(&c2..) */
  } /* for(&c1..) */
  return true;

error:
  return false;
} /* sanity_check_cross_consistency() */

bool base_cache_creator::sanity_check_cache_overlap(
    const cads::acache_vectoro& caches) const {
  for (auto& c1 : caches) {
    auto c1_xspan = c1->xrspan();
    auto c1_yspan = c1->yrspan();

    /* Check caches do not overlap */
    for (auto& c2 : caches) {
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

bool base_cache_creator::sanity_check_free_block_overlap(
    const cads::acache_vectoro& caches,
    const cds::block3D_vectorno& free_blocks) const {
  for (auto& c1 : caches) {
    auto c1_xspan = c1->xrspan();
    auto c1_yspan = c1->yrspan();

    /* Check caches against free blocks */
    for (auto& b : free_blocks) {
      ER_CHECK(!(c1_xspan.overlaps_with(b->xrspan()) &&
                 c1_yspan.overlaps_with(b->yrspan())),
               "Cache%d xspan=%s, yspan=%s overlaps block%d "
               "xspan=%s,yspan=%s",
               c1->id().v(),
               rcppsw::to_string(c1_xspan).c_str(),
               rcppsw::to_string(c1_yspan).c_str(),
               b->id().v(),
               rcppsw::to_string(b->xrspan()).c_str(),
               rcppsw::to_string(b->yrspan()).c_str());
    } /* for(&b..) */
  }   /* for(&c1..) */

  return true;

error:
  return false;
} /* sanity_check_free_block_overlap() */

bool base_cache_creator::sanity_check_block_cluster_overlap(
    const cads::acache_vectoro& caches,
    const cfds::block3D_cluster_vector& clusters) const {

  for (auto& cache : caches) {
    for (auto& cluster : clusters) {
      ER_CHECK(
          !(cache->xrspan().overlaps_with(cluster->xrspan()) &&
            cache->yrspan().overlaps_with(cluster->yrspan())),
          "Cache%d xspan=%s, yspan=%s overlaps cluster w/xspan=%s,yspan=%s",
          cache->id().v(),
          rcppsw::to_string(cache->xrspan()).c_str(),
          rcppsw::to_string(cache->yrspan()).c_str(),
          rcppsw::to_string(cluster->xrspan()).c_str(),
          rcppsw::to_string(cluster->yrspan()).c_str());
    } /* for(&cluster..) */
  }   /* for(&cache..) */
  return true;

error:
  return false;
} /* sanity_check_block_cluster_overlap() */

NS_END(support, fordyca);
