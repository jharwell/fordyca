/**
 * @file cell_perception.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
 *
 * This file is part of RCPPSW.
 *
 * RCPPSW is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * RCPPSW is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * RCPPSW.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/events/cell_perception.hpp"
#include "fordyca/representation/perceived_cell2D.hpp"
#include "fordyca/events/block_drop.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/cell_unknown.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cell_perception::cell_perception(const std::shared_ptr<rcppsw::common::er_server>& server,
                                 uint8_t cell_state,
                                 representation::cell_entity* entity) :
    m_cell_state(cell_state),
    m_entity(entity),
    m_server(server) {}


/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cell_perception::visit(representation::perceived_cell2D& cell) {
  events::block_drop drop(m_server,
                          dynamic_cast<representation::block*>(m_entity));
  events::cell_empty empty;
  events::cell_unknown unknown;

  switch (m_cell_state) {
    case representation::cell2D_fsm::ST_UNKNOWN:
      cell.cell().accept(unknown);
      break;
    case representation::cell2D_fsm::ST_EMPTY:
      cell.cell().accept(empty);
      break;
    case representation::cell2D_fsm::ST_HAS_BLOCK:
      assert(drop.block());
      cell.cell().accept(drop);
      break;
    default:
      break;
  } /* switch() */
  cell.update_density(1.0);
} /* visit() */

NS_END(events, fordyca);
