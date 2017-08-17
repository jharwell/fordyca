/**
 * @file arena_map.cpp
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
#include "fordyca/representation/arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
int arena_map::robot_on_block(const argos::CVector2& pos) {
  for (size_t i = 0; i < m_blocks.size(); ++i) {
    if (m_blocks[i].contains_point(pos)) {
      return i;
    }
  } /* for(i..) */
  return -1;
} /* robot_on_block() */

void arena_map::event_block_nest_drop(block& block) {
  block.event_nest_drop();
  distribute_block(block, false);
} /* event_block_nest_drop() */

void arena_map::event_block_pickup(block& block, size_t robot_index) {
  cell2D& cell = m_grid.access(block.discrete_loc().first,
                               block.discrete_loc().second);
  cell.event_empty();
  block.event_pickup(robot_index);
} /* event_block_pickup() */

void arena_map::distribute_block(block& block, bool first_time) {
  /* set previous location to empty */
  if (!first_time) {
    cell2D& cell = m_grid.access(block.discrete_loc().first,
                                 block.discrete_loc().second);
    cell.event_empty();
  }
  m_block_distributor.distribute_block(block, first_time);
  cell2D& cell = m_grid.access(block.discrete_loc().first,
                               block.discrete_loc().second);
  cell.event_has_block();
} /* distribute_block() */

void arena_map::distribute_blocks(bool first_time) {
  /* reset all state machines */
  for (size_t i = 0; i < m_blocks.size(); ++i) {
    distribute_block(m_blocks[i], first_time);
  } /* for(i..) */
} /* distribute_blocks() */

NS_END(representation, fordyca);
