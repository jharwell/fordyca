/**
 * @file cell_unknown.cpp
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
#include "fordyca/events/cell_unknown.hpp"
#include "fordyca/ds/cell2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cell_unknown::visit(ds::cell2D& cell) {
  cell.entity(nullptr);
  cell.fsm().accept(*this);
} /* visit() */

void cell_unknown::visit(fsm::cell2D_fsm& fsm) {
  /*
   * Much faster to check this than to send a redundant event to the
   * FSM. Doesn't matter for just a few robots, but it does when you have
   * dozens/hundreds.
   */
  if (fsm.state_is_known()) {
    fsm.event_unknown();
  }
} /* visit() */

NS_END(events, fordyca);
