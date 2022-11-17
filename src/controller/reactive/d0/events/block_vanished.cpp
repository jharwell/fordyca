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
#include "fordyca/controller/reactive/d0/events/block_vanished.hpp"

#include "fordyca/controller/reactive/d0/crw_controller.hpp"
#include "fordyca/fsm/d0/crw_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, reactive, d0, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_vanished::block_vanished(const rtypes::type_uuid& block_id)
    : ER_CLIENT_INIT("fordyca.controller.reactive.d0.events.block_vanished"),
      mc_block_id(block_id) {}

/*******************************************************************************
 * Controllers
 ******************************************************************************/
void block_vanished::visit(fcreactive::d0::crw_controller& controller) {
  controller.ndc_uuid_push();

  ER_INFO("Abort pickup: block%d vanished", mc_block_id.v());
  visit(*controller.fsm());

  controller.ndc_uuid_pop();
} /* visit() */

/*******************************************************************************
 * FSMs
 ******************************************************************************/
void block_vanished::visit(ffsm::d0::crw_fsm& fsm) {
  fsm.inject_event(ffsm::foraging_signal::ekBLOCK_VANISHED,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

NS_END(events, d0, reactive, controller, fordyca);
