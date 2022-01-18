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
