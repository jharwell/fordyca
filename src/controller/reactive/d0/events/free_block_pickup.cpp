/**
 * \file free_block_pickup.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/reactive/d0/events/free_block_pickup.hpp"

#include "cosm/repr/base_block3D.hpp"

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
free_block_pickup::free_block_pickup(crepr::base_block3D* block,
                                     const rtypes::type_uuid& id,
                                     const rtypes::timestep& t)
    : ER_CLIENT_INIT("fordyca.controller.reactive.d0.events.free_block_pickup"),
      ccops::base_block_pickup(block, id, t) {}

/*******************************************************************************
 * CRW Foraging
 ******************************************************************************/
void free_block_pickup::visit(fcreactive::d0::crw_controller& controller) {
  controller.ndc_uuid_push();

  visit(static_cast<ccontroller::block_carrying_controller&>(controller));
  visit(*controller.fsm());
  ER_INFO("Picked up block%d", block()->id().v());

  controller.ndc_uuid_pop();
} /* visit() */

void free_block_pickup::visit(ffsm::d0::crw_fsm& fsm) {
  auto old = fsm.current_state();
  ER_DEBUG("Visiting FSM: state=%d", old);
  fsm.inject_event(ffsm::foraging_signal::ekBLOCK_PICKUP,
                   rpfsm::event_type::ekNORMAL);
  auto _new = fsm.current_state();
  ER_DEBUG("Visited FSM: state=%d", _new);
  ER_ASSERT(old != _new, "FSM did not change state after pickup");
} /* visit() */

NS_END(events, d0, reactive, controller, fordyca);
