/**
 * @file arena_map.cpp
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
#include "fordyca/representation/arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void arena_map::distribute_blocks(void) {
  /* reset all state machines */
  m_grid.reset_cells(cell2D_fsm::EMPTY);

  /* assign blocks to new locations and update state machines */
  m_block_distributor.distribute_blocks(m_blocks);

  for (size_t i = 0; i < m_blocks.size(); ++i) {
    cell2D& cell = m_grid.access(m_blocks[i].discrete_loc().first,
                                 m_blocks[i].discrete_loc().second);
    cell.change_state(new cell2D_fsm::new_state_data(cell2D_fsm::BLOCK));
  } /* for(i..) */
} /* distribute_blocks() */

NS_END(representation, fordyca);
