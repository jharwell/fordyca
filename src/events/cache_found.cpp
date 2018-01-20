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
#include "fordyca/controller/depth1/foraging_controller.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/representation/cache.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_found::cache_found(const std::shared_ptr<rcppsw::er::server>& server,
                         std::unique_ptr<representation::cache> cache)
    : perceived_cell_op(cache->discrete_loc().first,
                        cache->discrete_loc().second),
      client(server),
      m_cache(std::move(cache)) {
  client::insmod("cache_found",
                 rcppsw::er::er_lvl::DIAG,
                 rcppsw::er::er_lvl::NOM);
}

cache_found::~cache_found(void) { client::rmmod(); }

/*******************************************************************************
 * Depth1 Foraging
 ******************************************************************************/
void cache_found::visit(representation::perceived_cell2D& cell) {
  /*
   * Update the pheromone density associated with the cell BEFORE updating the
   * state of the cell.
   *
   * It is possible that this cache found event was generated because we
   * previous had a block in our LOS and now the cell that it was on is now
   * occupied by a cache (either because we were near the static cache in the
   * arena, or because someone else dropped a block near to this one).In that
   * case, the cell will still be in the HAS_BLOCK state until the event is
   * propagated all the way through, and we can use that to reset the pheromone
   * density for the cell, which may be very high. Should be reset to 1.0 for a
   * newly discovered cache.
   */
  if (cell.state_has_block()) {
    cell.density_reset();
  }
  if (cell.pheromone_repeat_deposit() ||
      (!cell.pheromone_repeat_deposit() && !cell.state_has_cache())) {
    cell.pheromone_add(1.0);
  }
  cell.decoratee().accept(*this);
} /* visit() */

void cache_found::visit(representation::cell2D& cell) {
  cell.entity(m_tmp_cache);
  cell.fsm().accept(*this);
} /* visit() */

void cache_found::visit(fsm::cell2D_fsm& fsm) {
  /*
   * If there are more blocks in the cache than currently exist in the cell,
   * then other robots have dropped blocks in cache since the last time we saw
   * it. If there are fewer blocks in the cache than currently exist in the
   * cell, then other robots have picked up blocks from the cache since the last
   * time we saw it. In either case, synchronize the cell with the reality of
   * the cache we have just observed.
   */
  for (size_t i = fsm.block_count(); i < m_tmp_cache->n_blocks(); ++i) {
    fsm.event_block_drop();
  } /* for(i..) */

  for (size_t i = fsm.block_count(); i > m_tmp_cache->n_blocks(); --i) {
    fsm.event_block_pickup();
  } /* for(i..) */
} /* visit() */

void cache_found::visit(representation::perceived_arena_map& map) {
  representation::perceived_cell2D& cell =
      map.access(cell_op::x(), cell_op::y());

  /**
   * Remove any and all blocks from the known blocks list that exist in
   * the same space that a cache occupies.
   *
   * We can have blocks/caches overlapping (in terms of physical extent, not in
   * terms of cells), if we previously saw some of the leftover blocks when a
   * cache is destroyed, and left the area before a new cache could be
   * created. When we return to the arena and find a new cache there, we are
   * tracking blocks that no longer exist in our perception. Thus, the need for
   * this function.
   *
   * @note This is a hack, and once the robot computes its own LOS rather than
   * being sent it the need for this function will disappear.
   */
  auto it = map.blocks().begin();
  while (it != map.blocks().end()) {
    if (m_cache->contains_point((*it)->real_loc())) {
      ER_VER("Remove block%d hidden behind cache%d", (*it)->id(), m_cache->id());

      events::cell_empty op((*it)->discrete_loc().first,
                            (*it)->discrete_loc().second);
      map.access((*it)->discrete_loc().first, (*it)->discrete_loc().second)
          .accept(op);
      it = map.blocks().erase(it);
    } else {
      ++it;
    }
  } /* while(it..) */
  /*
   * If the cell is currently in a HAS_CACHE state, then that means that this
   * cell is coming back into our LOS with a block, when it contained a cache
   * the last time it was seen. Remove the cache/synchronize with reality.
   */
  if (cell.state_has_block()) {
    map.block_remove(cell.block());
  }
  m_tmp_cache = m_cache.get();
  map.cache_add(m_cache);
  cell.accept(*this);
} /* visit() */

NS_END(events, fordyca);
