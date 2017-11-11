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
cache_creator::cache_creator(std::shared_ptr<rcppsw::common::er_server> server,
                             representation::occupancy_grid& grid,
                             double cache_size, double resolution) :
    er_client(server),
    m_cache_size(cache_size),
    m_resolution(resolution),
    m_grid(grid),
    m_server(server) {
  er_client::insmod("cache_creator",
                    rcppsw::common::er_lvl::DIAG,
                    rcppsw::common::er_lvl::NOM);
    }

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
representation::cache cache_creator::create_single(
    std::list<representation::block*> blocks,
    const argos::CVector2& center) {

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
    events::free_block_drop op(m_server, block,
                               static_cast<size_t>(center.GetX()/ m_resolution),
                               static_cast<size_t>(center.GetY()/ m_resolution),
                               m_resolution);
    m_grid.access(op.x(), op.y()).accept(op);
  } /* for(block..) */
  ER_NOM("Create cache at (%f, %f) with  %zu blocks",
         center.GetX(), center.GetY(), blocks.size());

  representation::cache c(m_cache_size, center, blocks);
  c.discrete_loc(representation::real_to_discrete_coord(center, m_resolution));
  return c;
} /* create_single() */

NS_END(depth1, support, fordyca);
