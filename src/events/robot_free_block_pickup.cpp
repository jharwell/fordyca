/**
 * \file robot_free_block_pickup.cpp
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
#include "fordyca/events/robot_free_block_pickup.hpp"

#include "cosm/repr/base_block3D.hpp"

#include "fordyca/controller/reactive/d0/crw_controller.hpp"
#include "fordyca/controller/cognitive/d0/dpo_controller.hpp"
#include "fordyca/controller/cognitive/d0/mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d0/odpo_controller.hpp"
#include "fordyca/controller/cognitive/d0/omdpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_omdpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_omdpo_controller.hpp"
#include "fordyca/controller/cognitive/dpo_perception_subsystem.hpp"
#include "fordyca/controller/cognitive/mdpo_perception_subsystem.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/events/cell2D_empty.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/d0/crw_fsm.hpp"
#include "fordyca/fsm/d0/dpo_fsm.hpp"
#include "fordyca/fsm/d1/cached_block_to_nest_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/d0/generalist.hpp"
#include "fordyca/tasks/d1/harvester.hpp"
#include "fordyca/tasks/d2/cache_finisher.hpp"
#include "fordyca/tasks/d2/cache_starter.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);

using ds::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
robot_free_block_pickup::robot_free_block_pickup(crepr::base_block3D* block,
                                                 const rtypes::type_uuid& robot_id,
                                                 const rtypes::timestep& t)
    : ER_CLIENT_INIT("fordyca.events.robot_free_block_pickup"),
      ccops::base_block_pickup(block, robot_id, t) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void robot_free_block_pickup::dispatch_robot_free_block_interactor(
    tasks::base_foraging_task* const task) {
  auto* polled RCPPSW_UNUSED = dynamic_cast<cta::polled_task*>(task);
  auto* interactor = dynamic_cast<events::free_block_interactor*>(task);
  ER_ASSERT(nullptr != interactor,
            "Non free block interactor task %s causing free block pickup",
            polled->name().c_str());
  interactor->accept(*this);
} /* dispatch_robot_free_block_interactor() */

template <typename TController>
void robot_free_block_pickup::d1d2_dpo_controller_visit(
    TController& controller) {
  controller.ndc_pusht();

  visit(*controller.dpo_perception()->dpo_store());
  visit(static_cast<ccontroller::block_carrying_controller&>(controller));
  dispatch_robot_free_block_interactor(controller.current_task());

  controller.ndc_pop();
} /* d1d2_dpo_controller_visit() */

template <typename TController>
void robot_free_block_pickup::d1d2_mdpo_controller_visit(
    TController& controller) {
  controller.ndc_pusht();

  visit(*controller.mdpo_perception()->dpo_store());
  visit(static_cast<ccontroller::block_carrying_controller&>(controller));
  dispatch_robot_free_block_interactor(controller.current_task());

  controller.ndc_pop();
} /* d1d2_mdpo_controller_visit() */

template <typename TController>
void robot_free_block_pickup::d0_dpo_controller_visit(
    TController& controller) {
  controller.ndc_pusht();

  visit(*controller.dpo_perception()->dpo_store());
  visit(static_cast<ccontroller::block_carrying_controller&>(controller));
  visit(*controller.fsm());

  controller.ndc_pop();
} /* d0_dpo_controller_visit() */

template <typename TController>
void robot_free_block_pickup::d0_mdpo_controller_visit(
    TController& controller) {
  controller.ndc_pusht();

  visit(*controller.mdpo_perception()->map());
  visit(static_cast<ccontroller::block_carrying_controller&>(controller));
  visit(*controller.fsm());

  controller.ndc_pop();
} /* d0_mdpo_controller_visit() */

/*******************************************************************************
 * CRW Foraging
 ******************************************************************************/
void robot_free_block_pickup::visit(
    controller::reactive::d0::crw_controller& controller) {
  controller.ndc_pusht();

  visit(static_cast<ccontroller::block_carrying_controller&>(controller));
  visit(*controller.fsm());

  controller.ndc_pop();
} /* visit() */

void robot_free_block_pickup::visit(fsm::d0::crw_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_PICKUP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

/*******************************************************************************
 * DPO/MDPO Depth0 Foraging
 ******************************************************************************/
void robot_free_block_pickup::visit(ds::dpo_store& store) {
  ER_ASSERT(store.contains(block()),
            "Block%d not in DPO store",
            block()->id().v());
  store.block_remove(block());
  ER_ASSERT(!store.contains(block()),
            "Block%d in DPO store after removal",
            block()->id().v());
} /* visit() */

void robot_free_block_pickup::visit(ds::dpo_semantic_map& map) {
  cds::cell2D& cell =
      map.access<occupancy_grid::kCell>(cell2D_op::x(), cell2D_op::y());

  ER_ASSERT(block()->danchor2D() == cell.loc(),
            "Coordinates for block%d@%s/cell@%s do not agree",
            block()->id().v(),
            rcppsw::to_string(block()->danchor2D()).c_str(),
            rcppsw::to_string(cell.loc()).c_str());

  ER_ASSERT(block()->id() == cell.block3D()->id(),
            "Pickup/cell block mismatch: %d vs %d",
            block()->id().v(),
            cell.block3D()->id().v());
  /*
   * @bug: This should just be an assert. However, due to FORDYCA#242, the fact
   * that blocks can appear close to the wall, and FORDYCA#82, this may not
   * always be true (and the fact that it isn't is not an indication of
   * inconsistent simulation state :-( ). This can happen if, for example, a
   * robot is exploring for a block very near the edge of the arena, and happens
   * to drive over a block. In that case the block is not in its LOS (BUG!), or
   * in its occupancy grid, and hence the assertion failure here.
   */
  /* ER_ASSERT(cell.state_has_block(), "cell does not contain block"); */
  if (cell.state_has_block()) {
    map.block_remove(cell.block3D());
  }
} /* visit() */

void robot_free_block_pickup::visit(fsm::d0::dpo_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_PICKUP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void robot_free_block_pickup::visit(
    controller::cognitive::d0::mdpo_controller& controller) {
  d0_mdpo_controller_visit(controller);
} /* visit() */

void robot_free_block_pickup::visit(
    controller::cognitive::d0::omdpo_controller& controller) {
  d0_mdpo_controller_visit(controller);
} /* visit() */

void robot_free_block_pickup::visit(
    controller::cognitive::d0::dpo_controller& controller) {
  d0_dpo_controller_visit(controller);
} /* visit() */

void robot_free_block_pickup::visit(
    controller::cognitive::d0::odpo_controller& controller) {
  d0_dpo_controller_visit(controller);
} /* visit() */

/*******************************************************************************
 * DPO/MDPO Depth1 Foraging
 ******************************************************************************/
void robot_free_block_pickup::visit(
    controller::cognitive::d1::bitd_dpo_controller& controller) {
  d1d2_dpo_controller_visit(controller);
} /* visit() */

void robot_free_block_pickup::visit(
    controller::cognitive::d1::bitd_odpo_controller& controller) {
  d1d2_dpo_controller_visit(controller);
} /* visit() */

void robot_free_block_pickup::visit(
    controller::cognitive::d1::bitd_mdpo_controller& controller) {
  d1d2_mdpo_controller_visit(controller);
} /* visit() */

void robot_free_block_pickup::visit(
    controller::cognitive::d1::bitd_omdpo_controller& controller) {
  d1d2_mdpo_controller_visit(controller);
} /* visit() */

void robot_free_block_pickup::visit(tasks::d0::generalist& task) {
  visit(*static_cast<fsm::d0::free_block_to_nest_fsm*>(task.mechanism()));
} /* visit() */

void robot_free_block_pickup::visit(tasks::d1::harvester& task) {
  visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

void robot_free_block_pickup::visit(fsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_PICKUP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void robot_free_block_pickup::visit(fsm::d0::free_block_to_nest_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_PICKUP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

/*******************************************************************************
 * DPO/MDPO Depth2 Foraging
 ******************************************************************************/
void robot_free_block_pickup::visit(
    controller::cognitive::d2::birtd_dpo_controller& controller) {
  d1d2_dpo_controller_visit(controller);
} /* visit() */

void robot_free_block_pickup::visit(
    controller::cognitive::d2::birtd_mdpo_controller& controller) {
  d1d2_mdpo_controller_visit(controller);
} /* visit() */

void robot_free_block_pickup::visit(
    controller::cognitive::d2::birtd_odpo_controller& controller) {
  d1d2_dpo_controller_visit(controller);
} /* visit() */

void robot_free_block_pickup::visit(
    controller::cognitive::d2::birtd_omdpo_controller& controller) {
  d1d2_mdpo_controller_visit(controller);
} /* visit() */

void robot_free_block_pickup::visit(tasks::d2::cache_starter& task) {
  visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

void robot_free_block_pickup::visit(tasks::d2::cache_finisher& task) {
  visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

NS_END(detail, events, fordyca);
