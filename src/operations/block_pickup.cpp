/**
 * @file block_pickup.cpp
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
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/arena_map.hpp"
#include "fordyca/representation/perceived_cell2D.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/operations/block_pickup.hpp"
#include "fordyca/operations/cell_empty.hpp"
#include "fordyca/operations/cell_perception.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, operations);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_pickup::block_pickup(const std::shared_ptr<rcppsw::common::er_server>& server,
                           representation::block* block, size_t robot_index) :
    er_client(server), m_robot_index(robot_index), m_block(block),
    m_server(server) {
    insmod("block_pickup",
         rcppsw::common::er_lvl::DIAG,
         rcppsw::common::er_lvl::NOM);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_pickup::visit(representation::arena_map& map) {
  representation::discrete_coord old_d(m_block->discrete_loc().first,
                                       m_block->discrete_loc().second);
  argos::CVector2 old_r(m_block->real_loc().GetX(),
                        m_block->real_loc().GetY());
  representation::cell2D& cell = map.access(old_d.first, old_d.second);
  operations::cell_empty op;
  cell.accept(op);
  m_block->accept(*this);
  ER_NOM("arena_map: fb%zu: block%d from (%f, %f) -> (%zu, %zu)", m_robot_index,
         m_block->id(),
         old_r.GetX(), old_r.GetY(),
         old_d.first, old_d.second);
} /* visit() */

void block_pickup::visit(representation::perceived_arena_map& map) {
  ER_NOM("perceived_arena_map: fb%zu picked up block%d from (%f, %f) -> (%zu, %zu)",
         m_robot_index,
         m_block->id(),
         m_block->real_loc().GetX(), m_block->real_loc().GetY(),
         m_block->discrete_loc().first, m_block->discrete_loc().second);
  representation::perceived_cell2D& cell = map.access(
      m_block->discrete_loc().first, m_block->discrete_loc().second);

  operations::cell_perception percept_op(m_server,
                                         representation::cell2D_fsm::ST_EMPTY);
  cell.accept(percept_op);
} /* visit() */

void block_pickup::visit(representation::block& block) {
  ER_NOM("block: block%d is now carried by fb%zu",
         m_block->id(), m_robot_index);

  block.add_carry();
  assert(-1 != block.id());
  block.robot_index(m_robot_index);

  /* Move block out of sight */
  block.real_loc(argos::CVector2(100.0, 100.0));
  block.discrete_loc(representation::discrete_coord(100, 100));
} /* visit() */

NS_END(operations, fordyca);
