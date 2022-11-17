/**
 * \file block_vanished.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/cognitive/d1/events/block_vanished.hpp"

#include "fordyca/controller/cognitive/d1/bitd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_omdpo_controller.hpp"
#include "fordyca/events/free_block_interactor.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/d1/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d1, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_vanished::block_vanished(const rtypes::type_uuid& block_id)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d1.events.block_vanished"),
      fccd0::events::block_vanished(block_id) {}

/*******************************************************************************
 * Controllers
 ******************************************************************************/
void block_vanished::visit(fccd1::bitd_dpo_controller& controller) {
  controller.ndc_uuid_push();

  dispatch_free_block_interactor(controller.current_task());

  controller.ndc_uuid_pop();
} /* visit() */

void block_vanished::visit(fccd1::bitd_mdpo_controller& controller) {
  controller.ndc_uuid_push();

  dispatch_free_block_interactor(controller.current_task());

  controller.ndc_uuid_pop();
} /* visit() */

void block_vanished::visit(fccd1::bitd_odpo_controller& controller) {
  controller.ndc_uuid_push();

  dispatch_free_block_interactor(controller.current_task());

  controller.ndc_uuid_pop();
} /* visit() */

void block_vanished::visit(fccd1::bitd_omdpo_controller& controller) {
  controller.ndc_uuid_push();

  dispatch_free_block_interactor(controller.current_task());

  controller.ndc_uuid_pop();
} /* visit() */

/*******************************************************************************
 * FSMs
 ******************************************************************************/
void block_vanished::visit(ffsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(ffsm::foraging_signal::ekBLOCK_VANISHED,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_vanished::dispatch_free_block_interactor(
    tasks::base_foraging_task* const task) {
  ER_INFO("Abort pickup while executing task %s: block%d vanished",
          dynamic_cast<cta::logical_task*>(task)->name().c_str(),
          block_id().v());
  auto* interactor = dynamic_cast<fevents::free_block_interactor*>(task);
  ER_ASSERT(nullptr != interactor,
            "Non-free block interactor task %s triggered block vanished event",
            dynamic_cast<cta::logical_task*>(task)->name().c_str());
  interactor->accept(*this);
} /* dispatch_free_block_interactor() */

NS_END(events, d1, cognitive, controller, fordyca);
