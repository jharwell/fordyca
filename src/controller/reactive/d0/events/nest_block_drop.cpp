/**
 * \file nest_block_drop.cpp
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
#include "fordyca/controller/reactive/d0/events/nest_block_drop.hpp"

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
nest_block_drop::nest_block_drop(crepr::base_block3D* block,
                                 const rtypes::timestep& t)
    : ER_CLIENT_INIT("fordyca.controller.reactive.d0.events.nest_block_drop"),
      mc_timestep(t),
      m_block(block) {}

/*******************************************************************************
 * CRW Foraging
 ******************************************************************************/
void nest_block_drop::visit(fcreactive::d0::crw_controller& controller) {
  controller.ndc_uuid_push();
  visit(*controller.fsm());

  ER_INFO("Dropped block%d in nest", m_block->id().v());
  controller.ndc_uuid_pop();
} /* visit() */

void nest_block_drop::visit(ffsm::d0::crw_fsm& fsm) {
  fsm.inject_event(ffsm::foraging_signal::ekBLOCK_DROP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

NS_END(events, d0, reactive, controller, fordyca);
