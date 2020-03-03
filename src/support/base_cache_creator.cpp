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

#include "cosm/foraging/events/arena_free_block_drop.hpp"
#include "cosm/foraging/events/cell2D_cache_extent.hpp"
#include "cosm/foraging/repr/arena_cache.hpp"
#include "cosm/foraging/repr/block_cluster.hpp"
#include "cosm/foraging/repr/light_type_index.hpp"

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
std::unique_ptr<cfrepr::arena_cache> base_cache_creator::create_single_cache(
    const rmath::vector2d& center,
    cfds::block_vectorno blocks,
    const rtypes::timestep& t) {
  ER_ASSERT(center.x() > 0 && center.y() > 0,
            "Center@%s is not positive definite",
            center.to_str().c_str());
  /*
   * The cell that will be the location of the new cache may already contain a
   * block. If so, it should be added to the list of blocks for the cache.
   */
  rmath::vector2u d = rmath::dvec2uvec(center, grid()->resolution().v());
  cds::cell2D& cell = m_grid->access<arena_grid::kCell>(d);
  if (cell.state_has_block()) {
    ER_ASSERT(nullptr != cell.block(),
              "Cell@%s does not have block",
              cell.loc().to_str().c_str());

    /*
     * If the host cell for the cache already contains a block, we don't want to
     * unconditionally add said block to the list of blocks for the new cache,
     * because if the cache was created using some function of free blocks in
     * the arena, then that block could already be in the list, and we would be
     * double adding it, which will cause problems later (dynamic cache
     * creation). However, it may also NOT be in the list of blocks to use for
     * the new cache, in which case we need to add in (static cache creation).
     */
    if (blocks.end() == std::find(blocks.begin(), blocks.end(), cell.block())) {
      /*
       * We use insert() instead of push_back() here so that if there was a
       * leftover block on the cell where a cache used to be that is also where
       * this cache is being created, it becomes the "front" of the cache, and
       * will be the first block picked up by a robot from the new cache. This
       * helps to ensure fairness/better statistics for the simulations.
       */
      ER_DEBUG("Add block%d in from cache host cell@%s to block vector",
               cell.block()->id().v(),
               cell.loc().to_str().c_str());
      blocks.insert(blocks.begin(), cell.block());
    }
  }

  /*
   * We don't need to lock around the cell empty and block drop events because
   * cache creation always happens AFTER all robots have had their control steps
   * run, never DURING. We are not REALLY holding all the locks, but no need to
   * grab them in a non-concurrent context.
   */
  for (auto& block : blocks) {
    events::cell2D_empty_visitor op(block->dloc());
    op.visit(m_grid->access<arena_grid::kCell>(op.coord()));
  } /* for(block..) */

  for (auto& block : blocks) {
    cfevents::arena_free_block_drop_visitor op(
        block, d, m_grid->resolution(), cfds::arena_map_locking::ekALL_HELD);
    op.visit(m_grid->access<arena_grid::kCell>(op.coord()));
  } /* for(block..) */

  cfds::block_vectorno block_vec(blocks.begin(), blocks.end());
  auto ret = std::make_unique<cfrepr::arena_cache>(
      cfrepr::arena_cache::params{mc_cache_dim,
                                  m_grid->resolution(),
                                  center,
                                  block_vec,
                                  rtypes::constants::kNoUUID},
      cfrepr::light_type_index()[cfrepr::light_type_index::kCache]);
  ret->creation_ts(t);
  ER_INFO("Create cache%d@%s/%s, xspan=%s/%s,yspan=%s/%s with %zu blocks [%s]",
          ret->id().v(),
          ret->rloc().to_str().c_str(),
          ret->dloc().to_str().c_str(),
          ret->xspan().to_str().c_str(),
          rmath::drange2urange(ret->xspan(), m_grid->resolution().v())
              .to_str()
              .c_str(),
          ret->yspan().to_str().c_str(),
          rmath::drange2urange(ret->yspan(), m_grid->resolution().v())
              .to_str()
              .c_str(),
          ret->n_blocks(),
          rcppsw::to_string(blocks).c_str());
  return ret;
} /* create_single_cache() */

void base_cache_creator::update_host_cells(cfds::acache_vectoro& caches) {
  /*
   * To reset all cells covered by a cache's extent, we simply send them a
   * CACHE_EXTENT event. EXCEPT for the cell that hosted the actual cache,
   * because it is currently in the HAS_CACHE state as part of the cache
   * creation process and setting it here will trigger an assert later.
   */
  for (auto& cache : caches) {
    m_grid->access<arena_grid::kCell>(cache->dloc()).entity(cache.get());

    auto xspan = cache->xspan();
    auto yspan = cache->yspan();
    auto xmin =
        static_cast<uint>(std::ceil(xspan.lb() / m_grid->resolution().v()));
    auto xmax =
        static_cast<uint>(std::ceil(xspan.ub() / m_grid->resolution().v()));
    auto ymin =
        static_cast<uint>(std::ceil(yspan.lb() / m_grid->resolution().v()));
    auto ymax =
        static_cast<uint>(std::ceil(yspan.ub() / m_grid->resolution().v()));

    for (uint i = xmin; i < xmax; ++i) {
      for (uint j = ymin; j < ymax; ++j) {
        rmath::vector2u c = rmath::vector2u(i, j);
        auto& cell = m_grid->access<arena_grid::kCell>(i, j);
        ER_ASSERT(cache->contains_point(
                      rmath::uvec2dvec(c, m_grid->resolution().v())),
                  "Cache%d does not contain point (%u, %u) within its extent",
                  cache->id().v(),
                  i,
                  j);

        if (c != cache->dloc()) {
          ER_ASSERT(!cell.state_in_cache_extent(),
                    "Cell@(%u, %u) already in CACHE_EXTENT",
                    i,
                    j);
          cfevents::cell2D_cache_extent_visitor e(c, cache.get());
          e.visit(cell);
        }
      } /* for(j..) */
    }   /* for(i..) */
  }     /* for(cache..) */
} /* update_host_cells() */

bool base_cache_creator::creation_sanity_checks(
    const cfds::acache_vectoro& caches,
    const cfds::block_vectorno& free_blocks,
    const cfds::block_cluster_vector& clusters) const {
  /* check caches against each other and internally for consistency */
  for (auto& c1 : caches) {
    auto& cell = m_grid->access<arena_grid::kCell>(c1->dloc());
    ER_CHECK(cell.fsm().state_has_cache(),
             "Cell@%s not in HAS_CACHE state",
             cell.loc().to_str().c_str());
    ER_CHECK(c1->n_blocks() == cell.block_count(),
             "Cache/cell disagree on # of blocks: cache=%zu/cell=%zu",
             c1->n_blocks(),
             cell.block_count());
    auto c1_xspan = c1->xspan();
    auto c1_yspan = c1->yspan();

    /* Check caches do not overlap */
    for (auto& c2 : caches) {
      if (c1->id() == c2->id()) {
        continue;
      }
      auto c2_xspan = c2->xspan();
      auto c2_yspan = c2->yspan();

      ER_CHECK(!(c1_xspan.overlaps_with(c2_xspan) &&
                 c1_yspan.overlaps_with(c2_yspan)),
               "Cache%d xspan=%s, yspan=%s overlaps cache%d "
               "xspan=%s,yspan=%s",
               c1->id().v(),
               c1_xspan.to_str().c_str(),
               c1_yspan.to_str().c_str(),
               c2->id().v(),
               c2_xspan.to_str().c_str(),
               c2_yspan.to_str().c_str());
    } /* for(&c2..) */

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

    /* Check caches against free blocks */
    for (auto& b : free_blocks) {
      ER_CHECK(!(c1_xspan.overlaps_with(b->xspan()) &&
                 c1_yspan.overlaps_with(b->yspan())),
               "Cache%d xspan=%s, yspan=%s overlaps block%d "
               "xspan=%s,yspan=%s",
               c1->id().v(),
               c1_xspan.to_str().c_str(),
               c1_yspan.to_str().c_str(),
               b->id().v(),
               b->xspan().to_str().c_str(),
               b->yspan().to_str().c_str());
    } /* for(&b..) */
  }   /* for(&c1..) */

  /* Check caches against block clusters */
  for (auto& cache : caches) {
    for (auto& cluster : clusters) {
      ER_CHECK(
          !(cache->xspan().overlaps_with(cluster->xspan()) &&
            cache->yspan().overlaps_with(cluster->yspan())),
          "Cache%d xspan=%s, yspan=%s overlaps cluster w/xspan=%s,yspan=%s",
          cache->id().v(),
          cache->xspan().to_str().c_str(),
          cache->yspan().to_str().c_str(),
          cluster->xspan().to_str().c_str(),
          cluster->yspan().to_str().c_str());
    } /* for(&cluster..) */
  }   /* for(&cache..) */
  return true;

error:
  return false;
} /* creation_sanity_checks() */

NS_END(support, fordyca);
