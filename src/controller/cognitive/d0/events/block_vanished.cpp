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
#include "fordyca/controller/cognitive/d0/events/block_vanished.hpp"

#include "fordyca/controller/cognitive/d0/dpo_controller.hpp"
#include "fordyca/controller/cognitive/d0/mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d0/odpo_controller.hpp"
#include "fordyca/controller/cognitive/d0/omdpo_controller.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/d0/dpo_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/d0/generalist.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d0, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_vanished::block_vanished(const rtypes::type_uuid& block_id)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d0.events.block_vanished"),
      mc_block_id(block_id) {}

/*******************************************************************************
 * Controllers
 ******************************************************************************/
void block_vanished::visit(fccd0::dpo_controller& controller) {
  controller.ndc_uuid_push();

  ER_INFO("Abort pickup: block%d vanished", mc_block_id.v());
  visit(*controller.fsm());

  controller.ndc_uuid_pop();
} /* visit() */

void block_vanished::visit(fccd0::mdpo_controller& controller) {
  controller.ndc_uuid_push();

  ER_INFO("Abort pickup: block%d vanished", mc_block_id.v());
  visit(*controller.fsm());

  controller.ndc_uuid_pop();
} /* visit() */

void block_vanished::visit(fccd0::odpo_controller& controller) {
  controller.ndc_uuid_push();

  ER_INFO("Abort pickup: block%d vanished", mc_block_id.v());
  visit(*controller.fsm());

  controller.ndc_uuid_pop();
} /* visit() */

void block_vanished::visit(fccd0::omdpo_controller& controller) {
  controller.ndc_uuid_push();

  ER_INFO("Abort pickup: block%d vanished", mc_block_id.v());
  visit(*controller.fsm());

  controller.ndc_uuid_pop();
} /* visit() */

/*******************************************************************************
 * FSMs
 ******************************************************************************/
void block_vanished::visit(ffsm::d0::dpo_fsm& fsm) {
  fsm.inject_event(ffsm::foraging_signal::ekBLOCK_VANISHED,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void block_vanished::visit(fsm::d0::free_block_to_nest_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_VANISHED,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

NS_END(events, d0, cognitive, controller, fordyca);
