/**
 * @file base_cache_creator.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/base_cache_creator.hpp"
#include <chrono>

#include "fordyca/ds/cell2D.hpp"
#include "fordyca/events/cell_cache_extent.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/repr/arena_cache.hpp"
#include "fordyca/support/light_type_index.hpp"
#include "fordyca/support/loop_utils/loop_utils.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);
using ds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_cache_creator::base_cache_creator(ds::arena_grid* const grid,
                                       double cache_dim)
    : ER_CLIENT_INIT("fordyca.support.base_cache_creator"),
      mc_cache_dim(cache_dim),
      m_grid(grid),
      m_rng(std::chrono::system_clock::now().time_since_epoch().count()) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::unique_ptr<repr::arena_cache> base_cache_creator::create_single_cache(
    ds::block_list blocks,
    const rmath::vector2d& center,
    uint timestep) {
  ER_ASSERT(center.x() > 0 && center.y() > 0,
            "Center@%s is not positive definite",
            center.to_str().c_str());
  /*
   * The cell that will be the location of the new cache may already contain a
   * block. If so, it should be added to the list of blocks for the cache.
   */
  rmath::vector2u d = rmath::dvec2uvec(center, grid()->resolution());
  ds::cell2D& cell = m_grid->access<arena_grid::kCell>(d);
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
       * We use insert() instead of push_back() here so that it there was a
       * leftover block on the cell where a cache used to be that is also where
       * this cache is being created, it becomes the "front" of the cache, and
       * will be the first block picked up by a robot from the new cache. This
       * helps to ensure fairness/better statistics for the simulations.
       */
      blocks.insert(blocks.begin(), cell.block());
    }
  }

  /*
   * The cells for all blocks that will comprise the cache should be set to
   * cache extent, and all blocks be deposited in a single cell.
   */
  for (auto& block : blocks) {
    events::cell_empty_visitor op(block->dloc());
    op.visit(m_grid->access<arena_grid::kCell>(op.x(), op.y()));
  } /* for(block..) */

  for (auto& block : blocks) {
    events::free_block_drop_visitor op(block, d, m_grid->resolution());
    op.visit(m_grid->access<arena_grid::kCell>(op.x(), op.y()));
  } /* for(block..) */

  ds::block_vector block_vec(blocks.begin(), blocks.end());
  auto ret = rcppsw::make_unique<repr::arena_cache>(
      repr::arena_cache::params{
          mc_cache_dim, m_grid->resolution(), center, block_vec, -1},
      light_type_index()[light_type_index::kCache]);
  ret->creation_ts(timestep);
  ER_INFO("Create cache%d@%s/%s, xspan=%s,yspan=%s with %zu blocks [%s]",
          ret->id(),
          ret->rloc().to_str().c_str(),
          ret->dloc().to_str().c_str(),
          ret->xspan().to_str().c_str(),
          ret->yspan().to_str().c_str(),
          ret->n_blocks(),
          rcppsw::to_string(blocks).c_str());
  return ret;
} /* create_single_cache() */

void base_cache_creator::update_host_cells(ds::cache_vector& caches) {
  /*
   * To reset all cells covered by a cache's extent, we simply send them a
   * CACHE_EXTENT event. EXCEPT for the cell that hosted the actual cache,
   * because it is currently in the HAS_CACHE state as part of the cache
   * creation process and setting it here will trigger an assert later.
   */
  for (auto& cache : caches) {
    m_grid->access<arena_grid::kCell>(cache->dloc()).entity(cache);

    auto xspan = cache->xspan();
    auto yspan = cache->yspan();
    auto xmin = static_cast<uint>(std::ceil(xspan.lb() / m_grid->resolution()));
    auto xmax = static_cast<uint>(std::ceil(xspan.ub() / m_grid->resolution()));
    auto ymin = static_cast<uint>(std::ceil(yspan.lb() / m_grid->resolution()));
    auto ymax = static_cast<uint>(std::ceil(yspan.ub() / m_grid->resolution()));

    for (uint i = xmin; i < xmax; ++i) {
      for (uint j = ymin; j < ymax; ++j) {
        rmath::vector2u c = rmath::vector2u(i, j);
        if (c != cache->dloc()) {
          ER_ASSERT(cache->contains_point(
                        rmath::uvec2dvec(c, m_grid->resolution())),
                    "Cache%d does not contain point (%u, %u) within its extent",
                    cache->id(),
                    i,
                    j);
          auto& cell = m_grid->access<arena_grid::kCell>(i, j);
          ER_ASSERT(!cell.state_in_cache_extent(),
                    "Cell@(%u, %u) already in CACHE_EXTENT",
                    i,
                    j);
          events::cell_cache_extent_visitor e(c, cache);
          e.visit(cell);
        }
      } /* for(j..) */
    }   /* for(i..) */
  }     /* for(cache..) */
} /* update_host_cells() */

__rcsw_pure bool base_cache_creator::creation_sanity_checks(
    const ds::cache_vector& caches,
    const ds::block_list& free_blocks) const {
  for (auto& c1 : caches) {
    auto c1_xspan = c1->xspan();
    auto c1_yspan = c1->yspan();
    for (auto& c2 : caches) {
      if (c1->id() == c2->id()) {
        continue;
      }
      for (auto& b : c1->blocks()) {
        ER_CHECK(c1->blocks().end() == std::adjacent_find(c1->blocks().begin(),
                                                          c1->blocks().end()),
                 "Multiple blocks with the same ID in cache%d",
                 c1->id());

        ER_CHECK(!c2->contains_block(b),
                 "Block%d contained in both cache%d and cache%d",
                 b->id(),
                 c1->id(),
                 c2->id());
        auto c2_xspan = c2->xspan();
        auto c2_yspan = c2->yspan();
        ER_CHECK(!(c1_xspan.overlaps_with(c2_xspan) &&
                   c1_yspan.overlaps_with(c2_yspan)),
                 "Cache%d xspan=%s, yspan=%s overlaps cache%d "
                 "xspan=%s,yspan=%s",
                 c1->id(),
                 c1_xspan.to_str().c_str(),
                 c1_yspan.to_str().c_str(),
                 c2->id(),
                 c2_xspan.to_str().c_str(),
                 c2_yspan.to_str().c_str());
      } /* for(&b..) */
    }   /* for(&c2..) */

    for (auto& b : free_blocks) {
      auto b_xspan = b->xspan();
      auto b_yspan = b->yspan();

      ER_CHECK(!(c1_xspan.overlaps_with(b_xspan) &&
                 c1_yspan.overlaps_with(b_yspan)),
               "Cache%d xspan=%s, yspan=%s overlaps block%d "
               "xspan=%s,yspan=%s",
               c1->id(),
               c1_xspan.to_str().c_str(),
               c1_yspan.to_str().c_str(),
               b->id(),
               b_xspan.to_str().c_str(),
               b_yspan.to_str().c_str());
    } /* for(&b..) */
  }   /* for(&c1..) */
  return true;

error:
  return false;
} /* creation_sanity_checks() */

NS_END(support, fordyca);
