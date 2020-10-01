/**
 * \file robot_nest_block_drop.cpp
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
#include "fordyca/events/robot_nest_block_drop.hpp"

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
#include "fordyca/fsm/d0/crw_fsm.hpp"
#include "fordyca/fsm/d0/dpo_fsm.hpp"
#include "fordyca/fsm/d1/cached_block_to_nest_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/d0/generalist.hpp"
#include "fordyca/tasks/d1/collector.hpp"
#include "fordyca/tasks/d1/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
robot_nest_block_drop::robot_nest_block_drop(crepr::base_block3D* block,
                                             const rtypes::timestep& t)
    : ER_CLIENT_INIT("fordyca.events.robot_nest_block_drop"),
      mc_timestep(t),
      m_block(block) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void robot_nest_block_drop::dispatch_nest_interactor(
    tasks::base_foraging_task* const task) {
  RCSW_UNUSED auto* polled = dynamic_cast<cta::polled_task*>(task);
  auto interactor = dynamic_cast<events::nest_interactor*>(task);
  ER_ASSERT(nullptr != interactor,
            "Non nest-interactor task %s causing nest block drop",
            polled->name().c_str());
  interactor->accept(*this);
} /* dispatch_nest_interactor() */

/*******************************************************************************
 * Depth0 Foraging
 ******************************************************************************/
void robot_nest_block_drop::visit(controller::reactive::d0::crw_controller& controller) {
  controller.ndc_pusht();
  visit(*controller.fsm());

  ER_INFO("Dropped block%d in nest", m_block->id().v());
  controller.ndc_pop();
} /* visit() */

void robot_nest_block_drop::visit(fsm::d0::crw_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_DROP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void robot_nest_block_drop::visit(controller::cognitive::d0::dpo_controller& controller) {
  controller.ndc_pusht();

  visit(*controller.fsm());
  ER_INFO("Dropped block%d in nest", m_block->id().v());

  controller.ndc_pop();
} /* visit() */

void robot_nest_block_drop::visit(
    controller::cognitive::d0::odpo_controller& controller) {
  controller.ndc_pusht();

  visit(*controller.fsm());
  ER_INFO("Dropped block%d in nest", m_block->id().v());

  controller.ndc_pop();
} /* visit() */

void robot_nest_block_drop::visit(fsm::d0::dpo_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_DROP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void robot_nest_block_drop::visit(
    controller::cognitive::d0::mdpo_controller& controller) {
  controller.ndc_pusht();

  visit(*controller.fsm());
  ER_INFO("Dropped block%d in nest", m_block->id().v());

  controller.ndc_pop();
} /* visit() */

void robot_nest_block_drop::visit(
    controller::cognitive::d0::omdpo_controller& controller) {
  controller.ndc_pusht();

  visit(*controller.fsm());
  ER_INFO("Dropped block%d in nest", m_block->id().v());

  controller.ndc_pop();
} /* visit() */

/*******************************************************************************
 * Depth1 Foraging
 ******************************************************************************/
void robot_nest_block_drop::visit(
    controller::cognitive::d1::bitd_dpo_controller& controller) {
  controller.ndc_pusht();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", m_block->id().v());

  controller.ndc_pop();
} /* visit() */

void robot_nest_block_drop::visit(
    controller::cognitive::d1::bitd_odpo_controller& controller) {
  controller.ndc_pusht();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", m_block->id().v());

  controller.ndc_pop();
} /* visit() */

void robot_nest_block_drop::visit(
    controller::cognitive::d1::bitd_mdpo_controller& controller) {
  controller.ndc_pusht();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", m_block->id().v());

  controller.ndc_pop();
} /* visit() */

void robot_nest_block_drop::visit(
    controller::cognitive::d1::bitd_omdpo_controller& controller) {
  controller.ndc_pusht();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", m_block->id().v());

  controller.ndc_pop();
} /* visit() */

void robot_nest_block_drop::visit(tasks::d0::generalist& task) {
  visit(*static_cast<fsm::d0::free_block_to_nest_fsm*>(task.mechanism()));
} /* visit() */

void robot_nest_block_drop::visit(tasks::d1::collector& task) {
  visit(*static_cast<fsm::d1::cached_block_to_nest_fsm*>(task.mechanism()));
} /* visit() */

void robot_nest_block_drop::visit(fsm::d1::cached_block_to_nest_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_DROP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void robot_nest_block_drop::visit(fsm::d0::free_block_to_nest_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_DROP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void robot_nest_block_drop::visit(
    controller::cognitive::d2::birtd_mdpo_controller& controller) {
  controller.ndc_pusht();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", m_block->id().v());

  controller.ndc_pop();
} /* visit() */

void robot_nest_block_drop::visit(
    controller::cognitive::d2::birtd_dpo_controller& controller) {
  controller.ndc_pusht();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", m_block->id().v());

  controller.ndc_pop();
} /* visit() */

void robot_nest_block_drop::visit(
    controller::cognitive::d2::birtd_odpo_controller& controller) {
  controller.ndc_pusht();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", m_block->id().v());

  controller.ndc_pop();
} /* visit() */

void robot_nest_block_drop::visit(
    controller::cognitive::d2::birtd_omdpo_controller& controller) {
  controller.ndc_pusht();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", m_block->id().v());

  controller.ndc_pop();
} /* visit() */

NS_END(detail, events, fordyca);
