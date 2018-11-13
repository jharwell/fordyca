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

#include "fordyca/ds/cell2D.hpp"
#include "fordyca/events/cell_cache_extent.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/representation/arena_cache.hpp"

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
    : ER_CLIENT_INIT("fordyca.support.depth1.base_cache_creator"),
      m_cache_dim(cache_dim),
      m_grid(grid) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::unique_ptr<representation::arena_cache> base_cache_creator::create_single_cache(
    block_list blocks,
    const rmath::vector2d& center) {
  /*
   * The cell that will be the location of the new cache may already contain a
   * block. If so, it should be added to the list of blocks for the cache.
   */
  rcppsw::math::dcoord2 d = math::rcoord_to_dcoord(center, grid()->resolution());
  ds::cell2D& cell = m_grid->access<arena_grid::kCell>(d);
  if (cell.state_has_block()) {
    ER_ASSERT(cell.block(), "Cell does not have block");

    /*
     * We use insert() instead of push_back() here so that it there was a
     * leftover block on the cell where a cache used to be that is also where
     * this cache is being created, it becomes the "front" of the cache, and
     * will be the first block picked up by a robot from the new cache. This
     * helps to ensure fairness/better statistics for the simulations.
     */
    blocks.insert(blocks.begin(), cell.block());
  }

  /*
   * The cells for all blocks that will comprise the cache should be set to
   * cache extent, and all blocks be deposited in a single cell.
   */
  for (auto& block : blocks) {
    events::cell_empty op(block->discrete_loc());
    m_grid->access<arena_grid::kCell>(op.x(), op.y()).accept(op);
  } /* for(block..) */

  for (auto& block : blocks) {
    events::free_block_drop op(block, d, m_grid->resolution());
    m_grid->access<arena_grid::kCell>(op.x(), op.y()).accept(op);
  } /* for(block..) */

  std::string s =
      std::accumulate(blocks.begin(),
                      blocks.end(),
                      std::string(),
                      [&](const std::string& a,
                          const std::shared_ptr<representation::base_block>& b) {
                        return a + "b" + std::to_string(b->id()) + ",";
                      });
  ds::block_vector block_vec(blocks.begin(), blocks.end());
  auto ret = rcppsw::make_unique<representation::arena_cache>(
      m_cache_dim, m_grid->resolution(), center, block_vec, -1);
  ER_INFO("Create cache%d@%s [%u,%u], xspan=%s,yspan=%s with %zu blocks [%s]",
          ret->id(),
          ret->real_loc().to_str().c_str(),
          ret->discrete_loc().first,
          ret->discrete_loc().second,
          ret->xspan(ret->real_loc()).to_str().c_str(),
          ret->yspan(ret->real_loc()).to_str().c_str(),
          ret->n_blocks(),
          s.c_str());
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
    m_grid->access<arena_grid::kCell>(cache->discrete_loc()).entity(cache);

    auto xspan = cache->xspan(cache->real_loc());
    auto yspan = cache->yspan(cache->real_loc());
    uint xmin = static_cast<uint>(std::ceil(xspan.lb() / m_grid->resolution()));
    uint xmax = static_cast<uint>(std::ceil(xspan.ub() / m_grid->resolution()));
    uint ymin = static_cast<uint>(std::ceil(yspan.lb() / m_grid->resolution()));
    uint ymax = static_cast<uint>(std::ceil(yspan.ub() / m_grid->resolution()));

    for (uint i = xmin; i < xmax; ++i) {
      for (uint j = ymin; j < ymax; ++j) {
        rcppsw::math::dcoord2 c = rcppsw::math::dcoord2(i, j);
        if (c != cache->discrete_loc()) {
          ER_ASSERT(cache->contains_point(
                        math::dcoord_to_rcoord(c, m_grid->resolution())),
                    "Cache%d does not contain point (%u, %u) within its extent",
                    cache->id(),
                    i,
                    j);
          auto& cell = m_grid->access<arena_grid::kCell>(i, j);
          ER_ASSERT(!cell.state_in_cache_extent(),
                    "cell(%u, %u) already in CACHE_EXTENT",
                    i,
                    j);
          events::cell_cache_extent e(c, cache);
          cell.accept(e);
        }
      } /* for(j..) */
    }   /* for(i..) */
  }     /* for(cache..) */
} /* update_host_cells() */

NS_END(support, fordyca);
