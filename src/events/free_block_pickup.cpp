/**
 * @file free_block_pickup.cpp
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
#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/controller/base_perception_subsystem.hpp"
#include "fordyca/controller/depth0/stateful_foraging_controller.hpp"
#include "fordyca/controller/depth0/stateless_foraging_controller.hpp"
#include "fordyca/controller/depth1/foraging_controller.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/fsm/depth0/stateful_foraging_fsm.hpp"
#include "fordyca/fsm/depth0/stateless_foraging_fsm.hpp"
#include "fordyca/fsm/depth1/block_to_goal_fsm.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/representation/arena_map.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/tasks/depth0/generalist.hpp"
#include "fordyca/tasks/depth1/harvester.hpp"
#include "fordyca/tasks/depth2/cache_finisher.hpp"
#include "fordyca/tasks/depth2/cache_starter.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

using representation::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
free_block_pickup::free_block_pickup(
    std::shared_ptr<rcppsw::er::server> server,
    std::shared_ptr<representation::base_block> block,
    uint robot_index,
    uint timestep)
    : cell_op(block->discrete_loc().first, block->discrete_loc().second),
      client(server),
      m_timestep(timestep),
      m_robot_index(robot_index),
      m_block(block),
      m_server(server) {
  client::insmod("free_block_pickup",
                 rcppsw::er::er_lvl::DIAG,
                 rcppsw::er::er_lvl::NOM);
}

/*******************************************************************************
 * Foraging Support
 ******************************************************************************/
void free_block_pickup::visit(fsm::cell2D_fsm& fsm) {
  fsm.event_block_pickup();
} /* visit() */

void free_block_pickup::visit(representation::cell2D& cell) {
  cell.fsm().accept(*this);
  cell.entity(nullptr);
  ER_NOM("cell2D: fb%u block%d from (%u, %u)",
         m_robot_index,
         m_block->id(),
         m_block->discrete_loc().first,
         m_block->discrete_loc().second);
} /* visit() */

void free_block_pickup::visit(representation::arena_map& map) {
  ER_ASSERT(m_block->discrete_loc() ==
                rcppsw::math::dcoord2(cell_op::x(), cell_op::y()),
            "FATAL: Coordinates for block/cell do not agree");
  argos::CVector2 old_r(m_block->real_loc().GetX(), m_block->real_loc().GetY());
  events::cell_empty op(cell_op::x(), cell_op::y());
  map.accept(op);
  m_block->accept(*this);
  ER_NOM("arena_map: fb%u: block%d from (%f, %f) -> (%zu, %zu)",
         m_robot_index,
         m_block->id(),
         old_r.GetX(),
         old_r.GetY(),
         cell_op::x(),
         cell_op::y());
} /* visit() */

/*******************************************************************************
 * Stateless Foraging
 ******************************************************************************/
void free_block_pickup::visit(representation::base_block& block) {
  ER_ASSERT(-1 != block.id(), "FATAL: Unamed block");
  block.add_transporter(m_robot_index);
  block.first_pickup_time(m_timestep);

  /* Move block out of sight */
  block.move_out_of_sight();
  ER_NOM("block: block%d is now carried by fb%u", m_block->id(), m_robot_index);
} /* visit() */

void free_block_pickup::visit(
    controller::depth0::stateless_foraging_controller& controller) {
  controller.fsm()->accept(*this);
  controller.block(m_block);
  controller.free_pickup_event(true);

  ER_NOM("stateless_foraging_controller: %s picked up block%d",
         controller.GetId().c_str(),
         m_block->id());
} /* visit() */

void free_block_pickup::visit(fsm::depth0::stateless_foraging_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_PICKUP,
                   state_machine::event_type::NORMAL);
} /* visit() */

/*******************************************************************************
 * Stateful Foraging
 ******************************************************************************/
void free_block_pickup::visit(representation::perceived_arena_map& map) {
  ER_ASSERT(m_block->discrete_loc() ==
                rcppsw::math::dcoord2(cell_op::x(), cell_op::y()),
            "FATAL: Coordinates for block/cell do not agree");
  representation::cell2D& cell =
      map.access<occupancy_grid::kCellLayer>(cell_op::x(), cell_op::y());

  /*
   * @bug: This should just be an assert. However, due to #242, the fact that
   * blocks can appear close to the wall, and rcppsw #82, this may not always be
   * true (and the fact that it isn't is not an indication of inconsistent
   * simulation state :-( ). This can happen if, for example, a robot is
   * exploring for a block very near the edge of the arena, and happens to drive
   * over a block. In that case the block is not in its LOS (BUG!), or it its
   * occupancy grid, and hence the assertion failure here.
   */
  /* ER_ASSERT(cell.state_has_block(), "FATAL: cell does not contain block"); */
  if (cell.state_has_block()) {
    map.block_remove(cell.block());
  }
} /* visit() */

void free_block_pickup::visit(fsm::depth0::stateful_foraging_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_PICKUP,
                   state_machine::event_type::NORMAL);
} /* visit() */

void free_block_pickup::visit(
    controller::depth0::stateful_foraging_controller& controller) {
  controller.perception()->map()->accept(*this);
  static_cast<tasks::depth0::foraging_task*>(controller.current_task())
      ->accept(*this);
  controller.block(m_block);
  ER_NOM("stateful_foraging_controller: %s picked up block%d",
         controller.GetId().c_str(),
         m_block->id());
} /* visit() */

/*******************************************************************************
 * Depth1 Foraging
 ******************************************************************************/
void free_block_pickup::visit(
    controller::depth1::foraging_controller& controller) {
  controller.perception()->map()->accept(*this);
  controller.block(m_block);
  auto depth0 =
      dynamic_cast<tasks::depth0::foraging_task*>(controller.current_task());
  auto depth1 =
      dynamic_cast<tasks::depth1::foraging_task*>(controller.current_task());
  if (nullptr != depth0) {
    depth0->accept(*this);
  } else if (nullptr != depth1) {
    depth1->accept(*this);
  }

  ER_NOM("depth1_foraging_controller: %s picked up block%d",
         controller.GetId().c_str(),
         m_block->id());
} /* visit() */

void free_block_pickup::visit(tasks::depth0::generalist& task) {
  static_cast<fsm::depth0::stateful_foraging_fsm*>(task.mechanism())
      ->accept(*this);
} /* visit() */

void free_block_pickup::visit(tasks::depth1::harvester& task) {
  static_cast<fsm::depth1::block_to_goal_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

void free_block_pickup::visit(fsm::depth1::block_to_goal_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_PICKUP,
                   state_machine::event_type::NORMAL);
} /* visit() */

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void free_block_pickup::visit(
    controller::depth2::foraging_controller& controller) {
  ER_ASSERT(false, "FATAL: Not implemented");
} /* visit() */

void free_block_pickup::visit(tasks::depth2::cache_starter& task) {
  static_cast<fsm::depth1::block_to_goal_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

void free_block_pickup::visit(tasks::depth2::cache_finisher& task) {
  static_cast<fsm::depth1::block_to_goal_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

NS_END(events, fordyca);
