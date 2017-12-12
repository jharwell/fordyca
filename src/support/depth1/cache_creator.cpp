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
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/representation/cell2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_creator::cache_creator(std::shared_ptr<rcppsw::er::server> server,
                             representation::occupancy_grid& grid,
                             double cache_size, double resolution) :
    client(server),
    m_cache_size(cache_size),
    m_resolution(resolution),
    m_grid(grid),
    m_server(server) {
  client::insmod("cache_creator",
                    rcppsw::er::er_lvl::DIAG,
                    rcppsw::er::er_lvl::NOM);
    }

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
representation::cache cache_creator::create_single(
    std::list<representation::block*> blocks,
    const argos::CVector2& center) {

  /*
   * The cell that will be the location of the new cache may already contain a
   * block. If so, it should be added to the list of blocks for the cache.
   */
  representation::discrete_coord d = representation::real_to_discrete_coord(center,
                                                                            m_resolution);
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
   * The cells for all blocks that will comprise the cache should be emptied,
   * and all blocks be deposited in a single cell.
   */
  for (auto block : blocks) {
    events::cell_empty op(block->discrete_loc().first,
                          block->discrete_loc().second);
    m_grid.access(op.x(), op.y()).accept(op);
  } /* for(block..) */

  for (auto block : blocks) {
    events::free_block_drop op(m_server, block, d.first, d.second, m_resolution);
    m_grid.access(op.x(), op.y()).accept(op);
  } /* for(block..) */
  ER_NOM("Create cache at (%f, %f) -> (%zu, %zu) with  %zu blocks",
         center.GetX(), center.GetY(), d.first, d.second, blocks.size());

  std::vector<representation::block*> blocks_list(blocks.begin(),
                                                  blocks.end());
  representation::cache c(m_cache_size, m_grid.resolution(),
                          center, blocks_list);
  c.discrete_loc(representation::real_to_discrete_coord(center, m_resolution));
  return c;
} /* create_single() */

void cache_creator::update_host_cells(std::vector<representation::cache>& caches) {
  for (auto& cache : caches) {
    m_grid.access(cache.discrete_loc().first,
                  cache.discrete_loc().second).entity(&cache);
  } /* for(cache..) */
} /* update_host_cells() */

NS_END(depth1, support, fordyca);
