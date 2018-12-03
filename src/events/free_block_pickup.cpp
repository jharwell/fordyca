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
#include "fordyca/controller/depth0/crw_controller.hpp"
#include "fordyca/controller/depth0/stateful_controller.hpp"
#include "fordyca/controller/depth1/greedy_partitioning_controller.hpp"
#include "fordyca/controller/depth2/greedy_recpart_controller.hpp"
#include "fordyca/ds/arena_map.hpp"
#include "fordyca/ds/perceived_arena_map.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/depth0/crw_fsm.hpp"
#include "fordyca/fsm/depth0/stateful_fsm.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/tasks/depth0/generalist.hpp"
#include "fordyca/tasks/depth1/harvester.hpp"
#include "fordyca/tasks/depth2/cache_finisher.hpp"
#include "fordyca/tasks/depth2/cache_starter.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

using ds::occupancy_grid;
namespace rfsm = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
free_block_pickup::free_block_pickup(
    std::shared_ptr<representation::base_block> block,
    uint robot_index,
    uint timestep)
    : cell_op(block->discrete_loc()),
      ER_CLIENT_INIT("fordyca.events.free_block_pickup"),
      m_timestep(timestep),
      m_robot_index(robot_index),
      m_block(block) {}

/*******************************************************************************
 * Foraging Support
 ******************************************************************************/
void free_block_pickup::visit(fsm::cell2D_fsm& fsm) {
  fsm.event_block_pickup();
} /* visit() */

void free_block_pickup::visit(ds::cell2D& cell) {
  cell.fsm().accept(*this);
  cell.entity(nullptr);
  ER_INFO("cell2D: fb%u block%d from %s",
          m_robot_index,
          m_block->id(),
          m_block->discrete_loc().to_str().c_str());
} /* visit() */

void free_block_pickup::visit(ds::arena_map& map) {
  ER_ASSERT(m_block->discrete_loc() ==
                rmath::vector2u(cell_op::x(), cell_op::y()),
            "Coordinates for block/cell do not agree");
  rmath::vector2d old_r = m_block->real_loc();
  events::cell_empty op(cell_op::coord());
  map.accept(op);
  m_block->accept(*this);
  ER_INFO("arena_map: fb%u: block%d@%s/%s",
          m_robot_index,
          m_block->id(),
          old_r.to_str().c_str(),
          cell_op::coord().to_str().c_str());
} /* visit() */

/*******************************************************************************
 * Stateless Foraging
 ******************************************************************************/
void free_block_pickup::visit(representation::base_block& block) {
  ER_ASSERT(-1 != block.id(), "Unamed block");
  block.add_transporter(m_robot_index);
  block.first_pickup_time(m_timestep);

  block.move_out_of_sight();
  ER_INFO("Block%d is now carried by fb%u", m_block->id(), m_robot_index);
} /* visit() */

void free_block_pickup::visit(controller::depth0::crw_controller& controller) {
  controller.ndc_push();
  controller.fsm()->accept(*this);
  controller.block(m_block);
  controller.free_pickup_event(true);

  ER_INFO("Picked up block%d", m_block->id());
  controller.ndc_pop();
} /* visit() */

void free_block_pickup::visit(fsm::depth0::crw_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_PICKUP,
                   rfsm::event_type::NORMAL);
} /* visit() */

/*******************************************************************************
 * Stateful Foraging
 ******************************************************************************/
void free_block_pickup::visit(ds::perceived_arena_map& map) {
  ER_ASSERT(m_block->discrete_loc() ==
                rmath::vector2u(cell_op::x(), cell_op::y()),
            "Coordinates for block/cell do not agree");
  ds::cell2D& cell =
      map.access<occupancy_grid::kCell>(cell_op::x(), cell_op::y());

  /*
   * @bug: This should just be an assert. However, due to #242, the fact that
   * blocks can appear close to the wall, and rcppsw #82, this may not always be
   * true (and the fact that it isn't is not an indication of inconsistent
   * simulation state :-( ). This can happen if, for example, a robot is
   * exploring for a block very near the edge of the arena, and happens to drive
   * over a block. In that case the block is not in its LOS (BUG!), or in its
   * occupancy grid, and hence the assertion failure here.
   */
  /* ER_ASSERT(cell.state_has_block(), "cell does not contain block"); */
  if (cell.state_has_block()) {
    map.block_remove(cell.block());
  }
} /* visit() */

void free_block_pickup::visit(fsm::depth0::stateful_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_PICKUP,
                   rfsm::event_type::NORMAL);
} /* visit() */

void free_block_pickup::visit(
    controller::depth0::stateful_controller& controller) {
  controller.ndc_push();
  controller.perception()->map()->accept(*this);
  controller.fsm()->accept(*this);
  controller.block(m_block);
  controller.free_pickup_event(true);
  ER_INFO("Picked up block%d", m_block->id());
  controller.ndc_pop();
} /* visit() */

/*******************************************************************************
 * Depth1 Foraging
 ******************************************************************************/
void free_block_pickup::visit(
    controller::depth1::greedy_partitioning_controller& controller) {
  controller.ndc_push();
  controller.perception()->map()->accept(*this);
  controller.free_pickup_event(true);
  controller.block(m_block);

  auto* polled = dynamic_cast<ta::polled_task*>(controller.current_task());
  auto* task =
      dynamic_cast<events::free_block_interactor*>(controller.current_task());
  ER_ASSERT(nullptr != task,
            "Non free block interactor task %s causing free block pickup",
            polled->name().c_str());

  task->accept(*this);
  ER_INFO("Picked up block%d", m_block->id());
  controller.ndc_pop();
} /* visit() */

void free_block_pickup::visit(tasks::depth0::generalist& task) {
  static_cast<fsm::depth0::free_block_to_nest_fsm*>(task.mechanism())
      ->accept(*this);
} /* visit() */

void free_block_pickup::visit(tasks::depth1::harvester& task) {
  static_cast<fsm::block_to_goal_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

void free_block_pickup::visit(fsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_PICKUP,
                   rfsm::event_type::NORMAL);
} /* visit() */

void free_block_pickup::visit(fsm::depth0::free_block_to_nest_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_PICKUP,
                   rfsm::event_type::NORMAL);
} /* visit() */

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void free_block_pickup::visit(
    controller::depth2::greedy_recpart_controller& controller) {
  controller.ndc_push();
  controller.perception()->map()->accept(*this);
  controller.free_pickup_event(true);
  controller.block(m_block);

  auto* polled = dynamic_cast<ta::polled_task*>(controller.current_task());
  auto* task =
      dynamic_cast<events::free_block_interactor*>(controller.current_task());
  ER_ASSERT(nullptr != task,
            "Non free block interactor task %s causing free block pickup",
            polled->name().c_str());

  task->accept(*this);
  ER_INFO("Picked up block%d", m_block->id());
  controller.ndc_pop();
} /* visit() */

void free_block_pickup::visit(tasks::depth2::cache_starter& task) {
  static_cast<fsm::block_to_goal_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

void free_block_pickup::visit(tasks::depth2::cache_finisher& task) {
  static_cast<fsm::block_to_goal_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

NS_END(events, fordyca);
