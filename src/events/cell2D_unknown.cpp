/**
 * \file cell2D_unknown.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
