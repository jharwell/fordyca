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

#include "cosm/arena/operations/block_extent_clear.hpp"
#include "cosm/arena/operations/cache_extent_set.hpp"
#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/arena/repr/light_type_index.hpp"
#include "cosm/ds/operations/cell2D_cache_extent.hpp"
#include "cosm/foraging/repr/block_cluster.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/repr/nest.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"

#include "fordyca/events/cell2D_empty.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);
using cds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_cache_creator::base_cache_creator(cds::arena_grid* const grid,
                                       const rtypes::spatial_dist& cache_dim,
                                       cfbd::base_distributor* block_distributor)
    : ER_CLIENT_INIT("fordyca.support.base_cache_creator"),
      mc_cache_dim(cache_dim),
      m_block_distributor(block_distributor),
      m_grid(grid) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::unique_ptr<carepr::arena_cache>
base_cache_creator::create_single_cache(const rmath::vector2d& center,
                                        cds::block3D_vectorno blocks,
                                        const rtypes::timestep& t,
                                        bool pre_dist) {
  ER_ASSERT(center.x() > 0 && center.y() > 0,
            "Center@%s is not positive definite",
            center.to_str().c_str());
  /*
   * The cell that will be the location of the new cache may already contain a
   * block. If so, it should be added to the list of blocks for the cache.
   */
  auto dcenter = rmath::dvec2zvec(center, m_grid->resolution().v());
  auto& cell = m_grid->access<arena_grid::kCell>(dcenter);
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
               rcppsw::to_string(dcenter).c_str());
      blocks.insert(blocks.begin(), cell.block3D());
    }
  }

  /*
   * If blocks have not been distributed yet, nothing to do.
   */
  if (!pre_dist) {
    ER_INFO("Clearing host cells and block extents for %zu blocks",
            blocks.size());
    /*
     * We don't need to lock around the cell empty and block drop events because
     * cache creation always happens AFTER all robots have had their control
     * steps run, never DURING. We are not REALLY holding all the locks, but no
     * need to grab them in a non-concurrent context.
     */
    for (auto& block : blocks) {
      ER_DEBUG("Clearing block%d host cell@%s,extents: x=%s,y=%s, removing from cluster",
               block->id().v(),
               rcppsw::to_string(block->danchor2D()).c_str(),
               rcppsw::to_string(block->xdspan()).c_str(),
               rcppsw::to_string(block->ydspan()).c_str());

      /* Mark block host cell as empty */
      events::cell2D_empty_visitor cell_op(block->danchor2D());
      cell_op.visit(m_grid->access<arena_grid::kCell>(block->danchor2D()));

      /* Clear block extent */
      caops::block_extent_clear_visitor block_op(block);
      block_op.visit(*m_grid);

      /* update parent block cluster with the "pickup" */
      m_block_distributor->cluster_update_after_pickup(block,
                                                       block->danchor2D());
    } /* for(block..) */
  }

  /*
   * Loop through all blocks and deposit them in the cache host cell, which
   * will be in the HAS_CACHE state after this loop, but will not yet have a
   * cache as its entity.
   *
   * We do NOT update block clusters if this cache happens to be in the middle
   * of one (e.g., random distributions), because if a block is in a cache, it
   * can't be in a cluster. We are creating a cache, so all the specificied
   * blocks will be in it, and therefore not part of a cache.
   */
  for (auto& block : blocks) {
    caops::free_block_drop_visitor op(block,
                                      dcenter,
                                      m_grid->resolution(),
                                      carena::arena_map_locking::ekALL_HELD,
                                      false);
    op.visit(m_grid->access<arena_grid::kCell>(op.coord()));
    op.visit(*block);
  } /* for(block..) */

  ER_DEBUG("All %zu blocks now in host cell%s",
           blocks.size(),
           rcppsw::to_string(dcenter).c_str());

  /* create the cache! */
  cds::block3D_vectorno block_vec(blocks.begin(), blocks.end());
  auto ret = std::make_unique<carepr::arena_cache>(
      carepr::arena_cache::params{ mc_cache_dim,
                                   m_grid->resolution(),
                                   center,
                                   block_vec,
                                   rtypes::constants::kNoUUID },
      carepr::light_type_index()[carepr::light_type_index::kCache]);
  ret->creation_ts(t);
  ER_ASSERT(dcenter == ret->dcenter2D(),
            "Created cache%d has bad center: %s != %s",
            ret->id().v(),
            rcppsw::to_string(ret->dcenter2D()).c_str(),
            rcppsw::to_string(dcenter).c_str());
  ER_INFO("Created cache%d@%s/%s,anchor=%s/%s,xspan=%s/%s,yspan=%s/%s with %zu "
          "blocks [%s]",
          ret->id().v(),
          rcppsw::to_string(ret->rcenter2D()).c_str(),
          rcppsw::to_string(ret->dcenter2D()).c_str(),
          rcppsw::to_string(ret->ranchor2D()).c_str(),
          rcppsw::to_string(ret->danchor2D()).c_str(),
          rcppsw::to_string(ret->xrspan()).c_str(),
          rcppsw::to_string(ret->xdspan()).c_str(),
          rcppsw::to_string(ret->yrspan()).c_str(),
          rcppsw::to_string(ret->ydspan()).c_str(),
          ret->n_blocks(),
          rcppsw::to_string(blocks).c_str());

  /* Update host cell */
  cell.entity(ret.get());
  cell.color(ret->color());

  return ret;
} /* create_single_cache() */

void base_cache_creator::cache_extents_configure(
    const cads::acache_vectoro& caches) {
  for (const auto & cache : caches) {
    caops::cache_extent_set_visitor e(cache.get());
    e.visit(*m_grid);
  } /* for(cache..) */
} /* cache_extents_configure() */

bool base_cache_creator::creation_sanity_checks(
    const cads::acache_vectorro& c_caches,
    const cds::block3D_vectorno& c_free_blocks,
    const cfds::block3D_cluster_vectorro& c_clusters,
    const cads::nest_vectorro& c_nests) const {
  for (const auto & c : c_caches) {
    ER_CHECK(sanity_check_internal_consistency(c),
             "Cache%d@%s/%s not internally consistent",
             c->id().v(),
             rcppsw::to_string(c->rcenter2D()).c_str(),
             rcppsw::to_string(c->dcenter2D()).c_str());
    ;
  }
  for (const auto & c : c_caches) {
    ER_CHECK(sanity_check_free_block_overlap(c, c_free_blocks),
             "One or more caches overlap with free blocks");
  } /* for(&c..) */

  for (const auto & c : c_caches) {
    ER_CHECK(sanity_check_block_cluster_overlap(c, c_clusters),
             "One or more caches overlap with one or more block clusters");
  } /* for(&c..) */

  for (const auto & c : c_caches) {
    ER_CHECK(sanity_check_nest_overlap(c, c_nests),
             "One or more caches overlap with one or more nests");
  } /* for(&c..) */

  ER_CHECK(sanity_check_cross_consistency(c_caches),
           "Two or more caches not cross-consistent");

  ER_CHECK(sanity_check_cache_overlap(c_caches), "Two or more caches overlap");

  return true;

error:
  return false;
} /* creation_sanity_checks() */

bool base_cache_creator::sanity_check_internal_consistency(
    const carepr::arena_cache* cache) const {
  auto& cell = m_grid->access<arena_grid::kCell>(cache->dcenter2D());
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
  for (const auto & pair : cache->blocks()) {
    auto* b = pair.second;
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

bool base_cache_creator::sanity_check_free_block_overlap(
    const carepr::arena_cache* cache,
    const cds::block3D_vectorno& free_blocks) const {
  auto xspan = cache->xrspan();
  auto yspan = cache->yrspan();

  for (const auto & b : free_blocks) {
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

bool base_cache_creator::sanity_check_block_cluster_overlap(
    const carepr::arena_cache* cache,
    const cfds::block3D_cluster_vectorro& clusters) const {
  for (const auto & cluster : clusters) {
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

bool base_cache_creator::sanity_check_nest_overlap(
    const carepr::arena_cache* cache,
    const cads::nest_vectorro& nests) const {
  for (const auto & nest : nests) {
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

bool base_cache_creator::sanity_check_cross_consistency(
    const cads::acache_vectorro& caches) const {
  for (const auto & c1 : caches) {
    auto c1_xspan = c1->xrspan();
    auto c1_yspan = c1->yrspan();

    /* check caches contain different blocks and no duplicates */
    for (const auto & c2 : caches) {
      if (c1->id() == c2->id()) {
        continue;
      }
      for (const auto & pair : c1->blocks()) {
        auto* b = pair.second;
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

bool base_cache_creator::sanity_check_cache_overlap(
    const cads::acache_vectorro& caches) const {
  for (const auto & c1 : caches) {
    auto c1_xspan = c1->xrspan();
    auto c1_yspan = c1->yrspan();

    /* Check caches do not overlap */
    for (const auto & c2 : caches) {
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
