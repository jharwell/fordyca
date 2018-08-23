/**
 * @file cache_creator.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/support/depth1/cache_creator.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/cell_cache_extent.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/representation/arena_cache.hpp"
#include "fordyca/representation/cell2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_creator::cache_creator(const std::shared_ptr<rcppsw::er::server>& server,
                             representation::arena_grid& grid,
                             double cache_size,
                             double resolution)
    : client(server),
      m_cache_size(cache_size),
      m_resolution(resolution),
      m_grid(grid) {
  client::insmod("cache_creator",
                 rcppsw::er::er_lvl::DIAG,
                 rcppsw::er::er_lvl::NOM);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::unique_ptr<representation::arena_cache> cache_creator::create_single(
    block_list blocks,
    const argos::CVector2& center) {
  /*
   * The cell that will be the location of the new cache may already contain a
   * block. If so, it should be added to the list of blocks for the cache.
   */
  rcppsw::math::dcoord2 d = math::rcoord_to_dcoord(center, m_resolution);
  representation::cell2D& cell = m_grid.access(d.first, d.second);
  if (cell.state_has_block()) {
    ER_ASSERT(cell.block(), "FATAL: Cell does not have block");

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
   * cache extent,
   * and all blocks be deposited in a single cell.
   */
  for (auto block : blocks) {
    events::cell_empty op(block->discrete_loc());
    m_grid.access(op.x(), op.y()).accept(op);
  } /* for(block..) */

  for (auto block : blocks) {
    events::free_block_drop op(client::server_ref(), block, d, m_resolution);
    m_grid.access(op.x(), op.y()).accept(op);
  } /* for(block..) */
  ER_NOM("Create cache at (%f, %f) -> (%u, %u) with  %zu blocks",
         center.GetX(),
         center.GetY(),
         d.first,
         d.second,
         blocks.size());

  block_vector block_vec(blocks.begin(), blocks.end());
  return rcppsw::make_unique<representation::arena_cache>(
      m_cache_size, m_grid.resolution(), center, block_vec, -1);
} /* create_single() */

void cache_creator::update_host_cells(cache_vector& caches) {
  /*
   * To reset all cells covered by a cache's extent, we simply send them a
   * CACHE_EXTENT event. EXCEPT for the cell that hosted the actual cache, because
   * it is currently in the HAS_CACHE state as part of the cache creation
   * process and setting it here will trigger an assert later.
   */
  for (auto& cache : caches) {
    m_grid.access(cache->discrete_loc()).entity(cache);
    auto xspan = cache->xspan(cache->real_loc());
    auto yspan = cache->yspan(cache->real_loc());
    for (size_t i = xspan.get_min() / m_resolution; i < xspan.get_max() / m_resolution; ++i) {
      for (size_t j = yspan.get_min() / m_resolution; j < yspan.get_max() / m_resolution; ++j) {
        if (rcppsw::math::dcoord2(i, j) != cache->discrete_loc()) {
          events::cell_cache_extent e(rcppsw::math::dcoord2(i, j), cache);
          m_grid.access(i, j).accept(e);
        }
      } /* for(j..) */
    } /* for(i..) */
  } /* for(cache..) */
} /* update_host_cells() */

NS_END(depth1, support, fordyca);
