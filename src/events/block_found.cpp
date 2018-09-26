/**
 * @file block_found.cpp
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
#include "fordyca/events/block_found.hpp"
#include "fordyca/ds/perceived_arena_map.hpp"
#include "fordyca/representation/base_block.hpp"
#include "rcppsw/swarm/pheromone_density.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);
using ds::occupancy_grid;
namespace swarm = rcppsw::swarm;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_found::block_found(std::unique_ptr<representation::base_block> block)
    : perceived_cell_op(block->discrete_loc().first,
                        block->discrete_loc().second),
      ER_CLIENT_INIT("fordyca.events.block_found"),
      m_block(std::move(block)) {}

/*******************************************************************************
 * Depth0 Foraging
 ******************************************************************************/
void block_found::visit(ds::cell2D& cell) {
  ER_ASSERT(nullptr != m_block, "nullptr block?");
  cell.entity(m_block);
  cell.fsm().accept(*this);
} /* visit() */

void block_found::visit(fsm::cell2D_fsm& fsm) {
  if (fsm.state_has_cache()) {
    for (size_t i = fsm.block_count(); i > 1; --i) {
      fsm.event_block_pickup();
    } /* for(i..) */
  } else if (!fsm.state_has_block()) {
    fsm.event_block_drop();
  }
  ER_ASSERT(fsm.state_has_block(),
            "Perceived cell in incorrect state after block found event");
} /* visit() */

void block_found::visit(ds::perceived_arena_map& map) {
  ds::cell2D& cell = map.access<occupancy_grid::kCell>(x(), y());
  swarm::pheromone_density& density = map.access<occupancy_grid::kPheromone>(x(),
                                                                             y());

  if (!cell.state_is_known()) {
    map.known_cells_inc();
    ER_ASSERT(map.known_cell_count() <= map.xdsize() * map.ydsize(),
              "Known cell count (%u) >= arena dimensions (%ux%u)",
              map.known_cell_count(),
              map.xdsize(),
              map.ydsize());
  }
  /*
   * If the cell is currently in a HAS_CACHE state, then that means that this
   * cell is coming back into our LOS with a block, when it contained a cache
   * the last time it was seen. Remove the cache/synchronize with reality.
   *
   * The density needs to be reset as well, as we are now tracking a different
   * kind of cell entity.
   */
  if (cell.state_has_cache()) {
    map.cache_remove(cell.cache());
  }

  /*
   * If the ID of the block we currently think resides in the cell and the ID of
   * the one we just found that actually resides there are not the same, we need
   * to reset the density for the cell, and start a new decay count.
   */
  if (cell.state_has_block() && cell.block()->id() != m_block->id()) {
    density.reset();
  }

  if (map.pheromone_repeat_deposit()) {
    density.pheromone_add(1.0);
  } else {
    /*
     * Seeing a new block on empty square or one that used to contain a cache.
     */
    if (!cell.state_has_block()) {
      density.reset();
      density.pheromone_add(1.0);
    } else { /* Seeing a known block again--set its relevance to the max */
      density.pheromone_set(1.0);
    }
  }
  /*
   * ONLY if we actually added a block the list of known blocks do we update
   * what block the cell points to. If we do it unconditionally, we are left
   * with dangling references as a result of mixing unique_ptr and raw ptr. See
   * #229.
   */
  if (map.block_add(m_block)) {
    /*
     * The density of the cell for the newly discovered block needs to be
     * reset, as the cell may have contained a different cache/block which no
     * longer exists, and we need to start a new density decay count for the
     * newly discovered block.
     */
    cell.accept(*this);
  }
} /* visit() */

NS_END(events, fordyca);
