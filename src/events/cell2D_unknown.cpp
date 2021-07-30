/**
 * \file cell2D_unknown.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/events/cell2D_unknown.hpp"

#include "fordyca/subsystem/perception/ds/occupancy_grid.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cell2D_unknown::visit(fspds::occupancy_grid& grid) {
  cds::cell2D& cell = grid.access<fspds::occupancy_grid::kCell>(x(), y());

  if (cell.state_is_known()) {
    ER_ASSERT(grid.known_cell_count() >= 1,
              "Known cell count (%zu) < 1 before event",
              grid.known_cell_count());

    grid.known_cells_dec();
  }
  visit(cell);
} /* visit() */

NS_END(detail, events, fordyca);
