/**
 * @file free_block_drop.cpp
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
#include "fordyca/events/free_block_drop.hpp"
#include <argos/core/utility/math/vector2.h>

#include "fordyca/representation/block.hpp"
#include "fordyca/representation/cell2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
free_block_drop::free_block_drop(
    const std::shared_ptr<rcppsw::er::server>& server,
    representation::block* block, size_t x, size_t y, double resolution) :
    cell_op(x, y),
    client(server),
    m_resolution(resolution),
    m_block(block) {
  client::insmod("free_block_drop",
                    rcppsw::er::er_lvl::DIAG,
                    rcppsw::er::er_lvl::NOM);
}

/*******************************************************************************
 * Foraging Support
 ******************************************************************************/
void free_block_drop::visit(representation::cell2D& cell) {
  cell.entity(m_block);
  m_block->accept(*this);
  cell.fsm().accept(*this);
} /* visit() */

void free_block_drop::visit(fsm::cell2D_fsm& fsm) {
  fsm.event_block_drop();
} /* visit() */

void free_block_drop::visit(representation::block& block) {
  block.reset();
  representation::discrete_coord d(cell_op::x(),cell_op::y());
  block.real_loc(representation::discrete_to_real_coord(d, m_resolution));
  block.discrete_loc(d);
} /* visit() */

NS_END(events, fordyca);
