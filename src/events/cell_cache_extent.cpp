/**
 * \file cell_cache_extent.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/events/cell_cache_extent.hpp"

#include "fordyca/ds/arena_map.hpp"
#include "fordyca/ds/cell2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);
using ds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cell_cache_extent::cell_cache_extent(
    const rmath::vector2u& coord,
    const std::shared_ptr<repr::base_cache>& cache)
    : cell_op(coord), m_cache(cache) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cell_cache_extent::visit(ds::cell2D& cell) {
  cell.entity(m_cache);
  visit(cell.fsm());
} /* visit() */

void cell_cache_extent::visit(fsm::cell2D_fsm& fsm) {
  fsm.event_cache_extent();
} /* visit() */

void cell_cache_extent::visit(ds::arena_map& map) {
  visit(map.access<arena_grid::kCell>(cell_op::coord()));
} /* visit() */

NS_END(detail, events, fordyca);
