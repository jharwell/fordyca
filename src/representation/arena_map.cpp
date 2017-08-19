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
 * Constructors/Destructor
 ******************************************************************************/
arena_map::arena_map(const struct grid_params* params,
                     argos::CRange<argos::Real> nest_x,
                     argos::CRange<argos::Real> nest_y) :
    m_blocks(params->block.n_blocks,
             block(params->block.dimension)),
    m_block_distributor(params->resolution,
                        argos::CRange<argos::Real>(params->lower.GetX(),
                                                   params->upper.GetX()),
                        argos::CRange<argos::Real>(params->lower.GetY(),
                                                   params->upper.GetY()),
                        nest_x, nest_y,
                        &params->block),
    m_server(std::make_shared<rcppsw::common::er_server>()),
    m_grid(params) {
  deferred_init(m_server);
  insmod("arena_map");
  server_handle()->dbglvl(rcppsw::common::er_lvl::NOM);
  server_handle()->mod_dbglvl(er_id(), rcppsw::common::er_lvl::NOM);
  ER_NOM("%zu x %zu @ %f resolution", m_grid.xsize(), m_grid.ysize(),
         m_grid.resolution());
  for (size_t i = 0; i < m_blocks.size(); ++i) {
    m_blocks[i].id(i);
  } /* for(i..) */
    }

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
  distribute_block(block, false);
  int robot_index = block.robot_index();
  block.event_nest_drop();
  ER_NOM("fb%d dropped block%d in nest", robot_index, block.id());
} /* event_block_nest_drop() */

void arena_map::event_block_pickup(block& block, size_t robot_index) {
  block::discrete_coord old_d(block.discrete_loc().first,
                             block.discrete_loc().second);
  argos::CVector2 old_r(block.real_loc().GetX(),
                        block.real_loc().GetY());
  cell2D& cell = m_grid.access(old_d.first, old_d.second);
  cell.event_empty();
  block.event_pickup(robot_index);
  ER_NOM("fb%zu picked up block%d from (%f, %f) -> (%zu, %zu)",
         robot_index, block.id(),
         old_r.GetX(), old_r.GetY(),
         old_d.first, old_d.second);
} /* event_block_pickup() */

void arena_map::distribute_block(block& block, bool first_time) {
  m_block_distributor.distribute_block(block, first_time);
  cell2D& cell = m_grid.access(block.discrete_loc().first,
                               block.discrete_loc().second);
  cell.event_has_block(&block);
  ER_NOM("Block%d: real_loc=(%f, %f) discrete_loc=(%zu, %zu)",
         block.id(),
         block.real_loc().GetX(),
         block.real_loc().GetY(),
         block.discrete_loc().first,
         block.discrete_loc().second);
} /* distribute_block() */

void arena_map::distribute_blocks(bool first_time) {
  for (size_t i = 0; i < m_blocks.size(); ++i) {
    distribute_block(m_blocks[i], first_time);
  } /* for(i..) */

  /*
   * Once all blocks have been distributed, all cells that do not have blocks
   * are empty.
   */
  for (size_t i = 0; i < m_grid.xsize(); ++i) {
    for (size_t j = 0; j < m_grid.ysize(); ++j) {
      cell2D& cell = m_grid.access(i, j);
      if (!cell.state_has_block()) {
        cell.event_empty();
      }
    } /* for(j..) */
  } /* for(i..) */
} /* distribute_blocks() */

NS_END(representation, fordyca);
