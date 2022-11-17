/**
 * \file nest_block_drop.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/cognitive/d1/events/nest_block_drop.hpp"

#include "cosm/repr/base_block3D.hpp"

#include "fordyca/controller/cognitive/d1/bitd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_omdpo_controller.hpp"
#include "fordyca/events/nest_interactor.hpp"
#include "fordyca/fsm/d1/cached_block_to_nest_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/d1/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d1, events);

using base_drop = fccd0::events::nest_block_drop;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
nest_block_drop::nest_block_drop(crepr::base_block3D* block,
                                 const rtypes::timestep& t)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d1.events.nest_block_drop"),
      base_drop(block, t) {}

/*******************************************************************************
 * Controllers
 ******************************************************************************/
void nest_block_drop::visit(fccd1::bitd_dpo_controller& controller) {
  controller.ndc_uuid_push();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", block()->id().v());

  controller.ndc_uuid_pop();
} /* visit() */

void nest_block_drop::visit(fccd1::bitd_odpo_controller& controller) {
  controller.ndc_uuid_push();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", block()->id().v());

  controller.ndc_uuid_pop();
} /* visit() */

void nest_block_drop::visit(fccd1::bitd_mdpo_controller& controller) {
  controller.ndc_uuid_push();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", block()->id().v());

  controller.ndc_uuid_pop();
} /* visit() */

void nest_block_drop::visit(fccd1::bitd_omdpo_controller& controller) {
  controller.ndc_uuid_push();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", block()->id().v());

  controller.ndc_uuid_pop();
} /* visit() */

/*******************************************************************************
 * FSMs
 ******************************************************************************/
void nest_block_drop::visit(ffsm::d1::cached_block_to_nest_fsm& fsm) {
  fsm.inject_event(ffsm::foraging_signal::ekBLOCK_DROP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nest_block_drop::dispatch_nest_interactor(
    ftasks::base_foraging_task* const task) {
  RCPPSW_UNUSED auto* polled = dynamic_cast<cta::polled_task*>(task);
  auto* interactor = dynamic_cast<fevents::nest_interactor*>(task);
  ER_ASSERT(nullptr != interactor,
            "Non nest-interactor task %s causing nest block drop",
            polled->name().c_str());
  interactor->accept(*this);
} /* dispatch_nest_interactor() */

NS_END(events, d1, cognitive, controller, fordyca);
