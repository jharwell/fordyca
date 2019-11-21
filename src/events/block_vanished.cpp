/**
 * \file block_vanished.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/events/block_vanished.hpp"

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
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/depth0/crw_fsm.hpp"
#include "fordyca/fsm/depth0/dpo_fsm.hpp"
#include "fordyca/fsm/depth2/block_to_cache_site_fsm.hpp"
#include "fordyca/fsm/depth2/block_to_new_cache_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/depth0/generalist.hpp"
#include "fordyca/tasks/depth1/harvester.hpp"
#include "fordyca/tasks/depth2/cache_finisher.hpp"
#include "fordyca/tasks/depth2/cache_starter.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_vanished::block_vanished(uint block_id)
    : ER_CLIENT_INIT("fordyca.events.block_vanished"), m_block_id(block_id) {}

void block_vanished::dispatch_free_block_interactor(
    tasks::base_foraging_task* const task) {
  ER_INFO("Abort pickup executing task %s: block%d vanished",
          dynamic_cast<rta::logical_task*>(task)->name().c_str(),
          m_block_id);
  auto* interactor = dynamic_cast<events::free_block_interactor*>(task);
  ER_ASSERT(nullptr != interactor,
            "Non-free block interactor task %s triggered block vanished event",
            dynamic_cast<rta::logical_task*>(task)->name().c_str());
  interactor->accept(*this);
} /* dispatch_free_block_interactor() */

/*******************************************************************************
 * Depth0 Foraging
 ******************************************************************************/
void block_vanished::visit(controller::depth0::crw_controller& controller) {
  controller.ndc_push();

  ER_INFO("Abort pickup: block%d vanished", m_block_id);
  visit(*controller.fsm());

  controller.ndc_pop();
} /* visit() */

void block_vanished::visit(controller::depth0::dpo_controller& controller) {
  controller.ndc_push();

  ER_INFO("Abort pickup: block%d vanished", m_block_id);
  visit(*controller.fsm());

  controller.ndc_pop();
} /* visit() */

void block_vanished::visit(controller::depth0::mdpo_controller& controller) {
  controller.ndc_push();

  ER_INFO("Abort pickup: block%d vanished", m_block_id);
  visit(*controller.fsm());

  controller.ndc_pop();
} /* visit() */

void block_vanished::visit(controller::depth0::odpo_controller& controller) {
  controller.ndc_push();

  ER_INFO("Abort pickup: block%d vanished", m_block_id);
  visit(*controller.fsm());

  controller.ndc_pop();
} /* visit() */

void block_vanished::visit(controller::depth0::omdpo_controller& controller) {
  controller.ndc_push();

  ER_INFO("Abort pickup: block%d vanished", m_block_id);
  visit(*controller.fsm());

  controller.ndc_pop();
} /* visit() */

void block_vanished::visit(fsm::depth0::crw_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_VANISHED,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void block_vanished::visit(fsm::depth0::dpo_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_VANISHED,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

/*******************************************************************************
 * Depth1 Foraging
 ******************************************************************************/
void block_vanished::visit(controller::depth1::bitd_dpo_controller& controller) {
  controller.ndc_push();

  dispatch_free_block_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void block_vanished::visit(controller::depth1::bitd_mdpo_controller& controller) {
  controller.ndc_push();

  dispatch_free_block_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void block_vanished::visit(controller::depth1::bitd_odpo_controller& controller) {
  controller.ndc_push();

  dispatch_free_block_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void block_vanished::visit(controller::depth1::bitd_omdpo_controller& controller) {
  controller.ndc_push();

  dispatch_free_block_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void block_vanished::visit(tasks::depth0::generalist& task) {
  this->visit(
      *static_cast<fsm::depth0::free_block_to_nest_fsm*>(task.mechanism()));
} /* visit() */

void block_vanished::visit(tasks::depth1::harvester& task) {
  this->visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

void block_vanished::visit(fsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_VANISHED,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void block_vanished::visit(fsm::depth0::free_block_to_nest_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_VANISHED,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void block_vanished::visit(controller::depth2::birtd_mdpo_controller& controller) {
  controller.ndc_push();

  dispatch_free_block_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void block_vanished::visit(controller::depth2::birtd_dpo_controller& controller) {
  controller.ndc_push();

  dispatch_free_block_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void block_vanished::visit(
    controller::depth2::birtd_omdpo_controller& controller) {
  controller.ndc_push();

  dispatch_free_block_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void block_vanished::visit(controller::depth2::birtd_odpo_controller& controller) {
  controller.ndc_push();

  dispatch_free_block_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void block_vanished::visit(tasks::depth2::cache_starter& task) {
  visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

void block_vanished::visit(tasks::depth2::cache_finisher& task) {
  visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

NS_END(detail, events, fordyca);
