/**
 * @file cache_found.cpp
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
#include "fordyca/events/cache_found.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/controller/depth1/foraging_controller.hpp"
#include "fordyca/representation/cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_found::cache_found(const std::shared_ptr<rcppsw::er::server>& server,
                         const representation::cache* cache, size_t x, size_t y) :
    perceived_cell_op(x, y),
    client(server),
    m_cache(cache) {
  client::insmod("cache_found",
                 rcppsw::er::er_lvl::VER,
                 rcppsw::er::er_lvl::NOM);
}

/*******************************************************************************
 * Depth1 Foraging
 ******************************************************************************/
void cache_found::visit(representation::perceived_cell2D& cell) {
  cell.add_pheromone(1.0);
  cell.update_density();
  cell.cell().accept(*this);
} /* visit() */

void cache_found::visit(representation::cell2D& cell) {
  cell.entity(const_cast<representation::cache*>(m_cache));
  cell.fsm().accept(*this);
} /* visit() */

void cache_found::visit(fsm::cell2D_fsm& fsm) {
  /*
   * If there are more blocks in the cache than currently exist in the cell,
   * then other robots have dropped blocks in cache since the last time we saw
   * it. If there are fewer blocks in the cache than currently exist in the
   * cell, then other robots have picked up blocks from the cache since the last
   * time we saw it. In either case, synchronize the cell with the cache,
   * because the cache reflects the reality that robots have just seen.
   */
  for (size_t i = fsm.block_count(); i < m_cache->n_blocks(); ++i) {
    fsm.event_block_drop();
  } /* for(i..) */

  for (size_t i = m_cache->n_blocks(); i > fsm.block_count(); --i) {
    fsm.event_block_pickup();
  } /* for(i..) */
} /* visit() */

void cache_found::visit(representation::perceived_arena_map& map) {
  map.access(cell_op::x(), cell_op::y()).accept(*this);
} /* visit() */

NS_END(events, fordyca);
