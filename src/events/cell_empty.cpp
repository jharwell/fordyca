/**
 * @file cell_empty.cpp
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
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "fordyca/representation/perceived_cell2D.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/representation/arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cell_empty::visit(representation::perceived_cell2D& cell) {
  cell.decoratee().accept(*this);
} /* visit() */

void cell_empty::visit(representation::cell2D& cell) {
  cell.entity(nullptr);
  cell.fsm().accept(*this);
} /* visit() */

void cell_empty::visit(fsm::cell2D_fsm& fsm) {
  fsm.event_empty();
} /* visit() */

void cell_empty::visit(representation::arena_map& map) {
  map.access(cell_op::x(), cell_op::y()).accept(*this);
} /* visit() */

void cell_empty::visit(representation::perceived_arena_map& map) {
  map.access(cell_op::x(), cell_op::y()).accept(*this);
} /* visit() */

NS_END(events, fordyca);
