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
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/controller/memory_foraging_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_found::block_found(const std::shared_ptr<rcppsw::common::er_server>& server,
                       representation::block* block) :
    er_client(server), m_block(block) {
  er_client::insmod("block_found",
                    rcppsw::common::er_lvl::DIAG,
                    rcppsw::common::er_lvl::NOM);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_found::visit(representation::perceived_cell2D& cell) {
  cell.add_pheromone(1.0);
  cell.update_density();
  cell.cell().accept(*this);
} /* visit() */

void block_found::visit(representation::cell2D& cell) {
  cell.entity(m_block);
  cell.fsm().accept(*this);
} /* visit() */

void block_found::visit(representation::cell2D_fsm& fsm) {
  fsm.event_block_drop();
} /* visit() */

void block_found::visit(controller::memory_foraging_controller& controller) {
  controller.map()->accept(*this);
  ER_NOM("memory_foraging_controller: %s found block%d",
         controller.GetId().c_str(), m_block->id());
} /* visit() */

void block_found::visit(representation::perceived_arena_map& map) {
  map.access(m_block->discrete_loc().first,
             m_block->discrete_loc().second).accept(*this);
} /* visit() */

NS_END(events, fordyca);
