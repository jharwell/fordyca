/**
 * \file free_block_pickup.cpp
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
#include "fordyca/events/free_block_pickup.hpp"

#include "fordyca/controller/depth0/crw_controller.hpp"
#include "fordyca/controller/depth0/dpo_controller.hpp"
#include "fordyca/controller/depth0/mdpo_controller.hpp"
#include "fordyca/controller/depth0/odpo_controller.hpp"
#include "fordyca/controller/depth0/omdpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_dpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_mdpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_odpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_omdpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_dpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_odpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_omdpo_controller.hpp"
#include "fordyca/controller/dpo_perception_subsystem.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/ds/arena_map.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/depth0/crw_fsm.hpp"
#include "fordyca/fsm/depth0/dpo_fsm.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/depth0/generalist.hpp"
#include "fordyca/tasks/depth1/harvester.hpp"
#include "fordyca/tasks/depth2/cache_finisher.hpp"
#include "fordyca/tasks/depth2/cache_starter.hpp"

#include "cosm/repr/base_block2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);

using ds::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
free_block_pickup::free_block_pickup(
    const std::shared_ptr<crepr::base_block2D>& block,
    uint robot_index,
    const rtypes::timestep& t)
    : ER_CLIENT_INIT("fordyca.events.free_block_pickup"),
      cell_op(block->dloc()),
      mc_timestep(t),
      mc_robot_index(robot_index),
      m_block(block) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void free_block_pickup::dispatch_free_block_interactor(
    tasks::base_foraging_task* const task) {
  RCSW_UNUSED auto* polled = dynamic_cast<rta::polled_task*>(task);
  auto* interactor = dynamic_cast<events::free_block_interactor*>(task);
  ER_ASSERT(nullptr != interactor,
            "Non free block interactor task %s causing free block pickup",
            polled->name().c_str());
  interactor->accept(*this);
} /* dispatch_free_block_interactor() */

/*******************************************************************************
 * CRW Foraging
 ******************************************************************************/
void free_block_pickup::visit(ds::arena_map& map) {
  ER_ASSERT(m_block->dloc() == rmath::vector2u(cell_op::x(), cell_op::y()),
            "Coordinates for block/cell do not agree");
  RCSW_UNUSED rmath::vector2d old_r = m_block->rloc();

  events::cell_empty_visitor op(cell_op::coord());
  map.grid_mtx().lock();
  op.visit(map);
  map.grid_mtx().unlock();

  /*
   * Already holding block mutex from \ref free_block_pickup_interactor, though
   * it is not necessary for block visitation for this event.
   */
  visit(*m_block);

  ER_INFO("arena_map: fb%u: block%d@%s/%s",
          mc_robot_index,
          m_block->id(),
          old_r.to_str().c_str(),
          cell_op::coord().to_str().c_str());
} /* visit() */

void free_block_pickup::visit(crepr::base_block2D& block) {
  ER_ASSERT(-1 != block.id(), "Unamed block");
  block.robot_pickup_event(mc_robot_index);
  block.first_pickup_time(mc_timestep);
  ER_INFO("Block%d is now carried by fb%u", m_block->id(), mc_robot_index);
} /* visit() */

void free_block_pickup::visit(controller::depth0::crw_controller& controller) {
  controller.ndc_push();
  visit(*controller.fsm());
  auto robot_block = m_block->clone();
  robot_block->robot_id(mc_robot_index);
  controller.block(std::move(robot_block));
  controller.block_manip_collator()->free_pickup_event(true);

  ER_INFO("Picked up block%d", m_block->id());
  controller.ndc_pop();
} /* visit() */

void free_block_pickup::visit(fsm::depth0::crw_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_PICKUP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

/*******************************************************************************
 * DPO/MDPO Depth0 Foraging
 ******************************************************************************/
void free_block_pickup::visit(ds::dpo_store& store) {
  ER_ASSERT(store.contains(m_block),
            "Block%d@%s not in DPO store",
            m_block->id(),
            m_block->dloc().to_str().c_str());
  store.block_remove(m_block);
  ER_ASSERT(!store.contains(m_block),
            "Block%d@%s in DPO store after removal",
            m_block->id(),
            m_block->dloc().to_str().c_str());
} /* visit() */

void free_block_pickup::visit(ds::dpo_semantic_map& map) {
  ds::cell2D& cell =
      map.access<occupancy_grid::kCell>(cell_op::x(), cell_op::y());

  ER_ASSERT(m_block->dloc() == cell.loc(),
            "Coordinates for block%d@%s/cell@%s do not agree",
            m_block->id(),
            m_block->dloc().to_str().c_str(),
            cell.loc().to_str().c_str());

  ER_ASSERT(m_block->id() == cell.block()->id(),
            "Pickup/cell block mismatch: %d vs %d",
            m_block->id(),
            cell.block()->id());
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

void free_block_pickup::visit(fsm::depth0::dpo_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_PICKUP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void free_block_pickup::visit(controller::depth0::mdpo_controller& controller) {
  controller.ndc_push();

  visit(*controller.mdpo_perception()->map());
  visit(*controller.fsm());

  auto robot_block = m_block->clone();
  robot_block->robot_id(mc_robot_index);
  controller.block(std::move(robot_block));

  controller.block_manip_collator()->free_pickup_event(true);
  ER_INFO("Picked up block%d", m_block->id());

  controller.ndc_pop();
} /* visit() */

void free_block_pickup::visit(controller::depth0::omdpo_controller& controller) {
  controller.ndc_push();

  visit(*controller.mdpo_perception()->map());
  visit(*controller.fsm());

  auto robot_block = m_block->clone();
  robot_block->robot_id(mc_robot_index);
  controller.block(std::move(robot_block));

  controller.block_manip_collator()->free_pickup_event(true);
  ER_INFO("Picked up block%d", m_block->id());

  controller.ndc_pop();
} /* visit() */

void free_block_pickup::visit(controller::depth0::dpo_controller& controller) {
  controller.ndc_push();

  visit(*controller.dpo_perception()->dpo_store());
  visit(*controller.fsm());

  auto robot_block = m_block->clone();
  robot_block->robot_id(mc_robot_index);
  controller.block(std::move(robot_block));

  controller.block_manip_collator()->free_pickup_event(true);
  ER_INFO("Picked up block%d", m_block->id());

  controller.ndc_pop();
} /* visit() */

void free_block_pickup::visit(controller::depth0::odpo_controller& controller) {
  controller.ndc_push();

  visit(*controller.dpo_perception()->dpo_store());
  visit(*controller.fsm());

  auto robot_block = m_block->clone();
  robot_block->robot_id(mc_robot_index);
  controller.block(std::move(robot_block));

  controller.block_manip_collator()->free_pickup_event(true);
  ER_INFO("Picked up block%d", m_block->id());

  controller.ndc_pop();
} /* visit() */

/*******************************************************************************
 * DPO/MDPO Depth1 Foraging
 ******************************************************************************/
void free_block_pickup::visit(
    controller::depth1::bitd_dpo_controller& controller) {
  controller.ndc_push();

  visit(*controller.dpo_perception()->dpo_store());
  controller.block_manip_collator()->free_pickup_event(true);

  auto robot_block = m_block->clone();
  robot_block->robot_id(mc_robot_index);
  controller.block(std::move(robot_block));

  dispatch_free_block_interactor(controller.current_task());
  ER_INFO("Picked up block%d", m_block->id());

  controller.ndc_pop();
} /* visit() */

void free_block_pickup::visit(
    controller::depth1::bitd_mdpo_controller& controller) {
  controller.ndc_push();

  visit(*controller.mdpo_perception()->map());
  controller.block_manip_collator()->free_pickup_event(true);

  auto robot_block = m_block->clone();
  robot_block->robot_id(mc_robot_index);
  controller.block(std::move(robot_block));

  dispatch_free_block_interactor(controller.current_task());
  ER_INFO("Picked up block%d", m_block->id());

  controller.ndc_pop();
} /* visit() */

void free_block_pickup::visit(
    controller::depth1::bitd_odpo_controller& controller) {
  controller.ndc_push();

  visit(*controller.dpo_perception()->dpo_store());
  controller.block_manip_collator()->free_pickup_event(true);

  auto robot_block = m_block->clone();
  robot_block->robot_id(mc_robot_index);
  controller.block(std::move(robot_block));

  dispatch_free_block_interactor(controller.current_task());
  ER_INFO("Picked up block%d", m_block->id());

  controller.ndc_pop();
} /* visit() */

void free_block_pickup::visit(
    controller::depth1::bitd_omdpo_controller& controller) {
  controller.ndc_push();

  visit(*controller.mdpo_perception()->map());
  controller.block_manip_collator()->free_pickup_event(true);

  auto robot_block = m_block->clone();
  robot_block->robot_id(mc_robot_index);
  controller.block(std::move(robot_block));

  dispatch_free_block_interactor(controller.current_task());
  ER_INFO("Picked up block%d", m_block->id());

  controller.ndc_pop();
} /* visit() */

void free_block_pickup::visit(tasks::depth0::generalist& task) {
  visit(*static_cast<fsm::depth0::free_block_to_nest_fsm*>(task.mechanism()));
} /* visit() */

void free_block_pickup::visit(tasks::depth1::harvester& task) {
  visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

void free_block_pickup::visit(fsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_PICKUP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void free_block_pickup::visit(fsm::depth0::free_block_to_nest_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_PICKUP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

/*******************************************************************************
 * DPO/MDPO Depth2 Foraging
 ******************************************************************************/
void free_block_pickup::visit(
    controller::depth2::birtd_dpo_controller& controller) {
  controller.ndc_push();

  visit(*controller.dpo_perception()->dpo_store());
  controller.block_manip_collator()->free_pickup_event(true);

  auto robot_block = m_block->clone();
  robot_block->robot_id(mc_robot_index);
  controller.block(std::move(robot_block));

  dispatch_free_block_interactor(controller.current_task());
  ER_INFO("Picked up block%d", m_block->id());

  controller.ndc_pop();
} /* visit() */

void free_block_pickup::visit(
    controller::depth2::birtd_mdpo_controller& controller) {
  controller.ndc_push();

  visit(*controller.mdpo_perception()->map());
  controller.block_manip_collator()->free_pickup_event(true);

  auto robot_block = m_block->clone();
  robot_block->robot_id(mc_robot_index);
  controller.block(std::move(robot_block));

  dispatch_free_block_interactor(controller.current_task());
  ER_INFO("Picked up block%d", m_block->id());

  controller.ndc_pop();
} /* visit() */

void free_block_pickup::visit(
    controller::depth2::birtd_odpo_controller& controller) {
  controller.ndc_push();

  visit(*controller.dpo_perception()->dpo_store());
  controller.block_manip_collator()->free_pickup_event(true);

  auto robot_block = m_block->clone();
  robot_block->robot_id(mc_robot_index);
  controller.block(std::move(robot_block));

  dispatch_free_block_interactor(controller.current_task());
  ER_INFO("Picked up block%d", m_block->id());

  controller.ndc_pop();
} /* visit() */

void free_block_pickup::visit(
    controller::depth2::birtd_omdpo_controller& controller) {
  controller.ndc_push();

  visit(*controller.mdpo_perception()->map());
  controller.block_manip_collator()->free_pickup_event(true);

  auto robot_block = m_block->clone();
  robot_block->robot_id(mc_robot_index);
  controller.block(std::move(robot_block));

  dispatch_free_block_interactor(controller.current_task());
  ER_INFO("Picked up block%d", m_block->id());

  controller.ndc_pop();
} /* visit() */

void free_block_pickup::visit(tasks::depth2::cache_starter& task) {
  visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

void free_block_pickup::visit(tasks::depth2::cache_finisher& task) {
  visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

NS_END(detail, events, fordyca);
