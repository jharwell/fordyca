/**
 * @file base_perception_subsystem.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/controller/base_perception_subsystem.hpp"
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/events/block_found.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
using representation::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_perception_subsystem::base_perception_subsystem(
    const std::shared_ptr<rcppsw::er::server>& server,
    const params::perception_params* const params,
    const std::string& id)
    : client(server),
      m_map(rcppsw::make_unique<representation::perceived_arena_map>(
          client::server_ref(),
          params,
          id)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_perception_subsystem::update(
    const representation::line_of_sight* const los) {
  process_los(los);
  m_map->update();
} /* update() */

void base_perception_subsystem::process_los(
    const representation::line_of_sight* const los) {
  /*
   * If the robot thinks that a cell contains a block, because the cell had one
   * the last time it passed nearby, but when coming near the cell a second time
   * the cell does not contain a block, then someone else picked up the block
   * between then and now, and it needs to update its internal representation
   * accordingly.
   */
  for (size_t i = 0; i < los->xsize(); ++i) {
    for (size_t j = 0; j < los->ysize(); ++j) {
      rcppsw::math::dcoord2 d = los->cell(i, j).loc();
      if (!los->cell(i, j).state_has_block() &&
          m_map->access<occupancy_grid::kCellLayer>(d).state_has_block()) {
        ER_DIAG("Correct block%d discrepency at (%zu, %zu)",
                m_map->access<occupancy_grid::kCellLayer>(d).block()->id(),
                d.first,
                d.second);
        m_map->block_remove(
            m_map->access<occupancy_grid::kCellLayer>(d).block());
      }
    } /* for(j..) */
  }   /* for(i..) */

  for (auto block : los->blocks()) {
    if (!m_map->access<occupancy_grid::kCellLayer>(block->discrete_loc())
             .state_has_block()) {
      ER_NOM("Discovered block%d at (%zu, %zu)",
             block->id(),
             block->discrete_loc().first,
             block->discrete_loc().second);
    }
    events::block_found op(client::server_ref(), block->clone());
    m_map->accept(op);
  } /* for(block..) */
} /* process_los() */

NS_END(controller, fordyca);
