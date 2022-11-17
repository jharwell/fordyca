/**
 * \file cell2D_empty.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/events/cell2D_empty.hpp"

#include "fordyca/subsystem/perception/ds/dpo_semantic_map.hpp"
#include "fordyca/subsystem/perception/ds/occupancy_grid.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cell2D_empty::visit(fspds::occupancy_grid& grid) {
  cds::cell2D& cell = grid.access<fspds::occupancy_grid::kCell>(x(), y());
  if (!cell.state_is_known()) {
    grid.known_cells_inc();
  }
  ER_ASSERT(grid.known_cell_count() <= grid.xdsize() * grid.ydsize(),
            "Known cell count (%zu) >= arena dimensions (%zux%zu)",
            grid.known_cell_count(),
            grid.xdsize(),
            grid.ydsize());
  grid.access<fspds::occupancy_grid::kPheromone>(x(), y()).reset();
  visit(cell);
} /* visit() */

void cell2D_empty::visit(fspds::dpo_semantic_map& map) {
  visit(map.decoratee());
} /* visit() */

NS_END(detail, events, fordyca);
