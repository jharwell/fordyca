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
#include "fordyca/controller/depth0/stateful_foraging_controller.hpp"
#include "fordyca/controller/depth1/foraging_controller.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_found::block_found(const std::shared_ptr<rcppsw::er::server> &server,
                         representation::block *block)
    : perceived_cell_op(block->discrete_loc().first,
                        block->discrete_loc().second),
      client(server),
      m_block(block) {
  client::insmod("block_found",
                 rcppsw::er::er_lvl::DIAG,
                 rcppsw::er::er_lvl::NOM);
}

/*******************************************************************************
 * Depth0 Foraging
 ******************************************************************************/
void block_found::visit(representation::cell2D &cell) {
  cell.entity(const_cast<representation::block *>(m_block));

  /*
   * We do not assert that the cell we found a block in does not have a cache,
   * because it is possible that we got into this function on the timestep that
   * another robot picked up the last block from a cache we are also on/in, and
   * the cell containing the cache in the actual arena is now just a block.
   *
   * We just update our view of the world to reflect the new information.
   */
  if (!cell.fsm().state_has_block()) {
    cell.fsm().accept(*this);
  }
} /* visit() */

void block_found::visit(fsm::cell2D_fsm &fsm) {
  if (fsm.state_has_cache()) {
    for (size_t i = fsm.block_count(); i > 1; --i) {
      fsm.event_block_pickup();
    } /* for(i..) */
  } else {
    fsm.event_block_drop();
  }
  ER_ASSERT(fsm.state_has_block(),
            "FATAL: Perceived cell does not contain block after block found "
            "event");
} /* visit() */

void block_found::visit(representation::perceived_cell2D &cell) {
  /*
   * Update the pheromone density associated with the cell BEFORE updating the
   * state of the cell.
   *
   * It is possible that this block found event was generated because we were
   * in/on a cache that someone else picked up the last block from, and the
   * remaining orphan block has now entered our LOS. In that case, the cell will
   * still be in the HAS_CACHE state until the event is propagated all the way
   * through, and we can use that to reset the pheromone density for the cell,
   * which may be very high (a well known cache). It should be reset to 1.0 for
   * a newly discovered block.
   */
  if (cell.state_has_cache()) {
    cell.density_reset();
  }

  if ((cell.state_has_block() && cell.pheromone_repeat_deposit()) ||
      !cell.state_has_block()) {
    cell.pheromone_add(1.0);
  }
  cell.decoratee().accept(*this);
} /* visit() */

void block_found::visit(representation::perceived_arena_map &map) {
  map.block_add(*m_block);
  m_block = &map.blocks().back();
  map.access(cell_op::x(), cell_op::y()).accept(*this);
} /* visit() */

NS_END(events, fordyca);
