/**
 * \file cell2D_empty.cpp
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
#include "fordyca/events/cell2D_empty.hpp"

#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/ds/occupancy_grid.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);
using ds::occupancy_grid;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cell2D_empty::visit(ds::occupancy_grid& grid) {
  cds::cell2D& cell = grid.access<occupancy_grid::kCell>(x(), y());
  if (!cell.state_is_known()) {
    grid.known_cells_inc();
  }
  ER_ASSERT(grid.known_cell_count() <= grid.xdsize() * grid.ydsize(),
            "Known cell count (%u) >= arena dimensions (%zux%zu)",
            grid.known_cell_count(),
            grid.xdsize(),
            grid.ydsize());
  grid.access<occupancy_grid::kPheromone>(x(), y()).reset();
  visit(cell);
} /* visit() */

void cell2D_empty::visit(ds::dpo_semantic_map& map) {
  visit(map.decoratee());
} /* visit() */

NS_END(detail, events, fordyca);
