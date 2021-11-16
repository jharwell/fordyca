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
#include "fordyca/controller/cognitive/d0/events/nest_block_drop.hpp"

#include "cosm/repr/base_block3D.hpp"

#include "fordyca/controller/cognitive/d0/dpo_controller.hpp"
#include "fordyca/controller/cognitive/d0/mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d0/odpo_controller.hpp"
#include "fordyca/controller/cognitive/d0/omdpo_controller.hpp"
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
nest_block_drop::nest_block_drop(crepr::base_block3D* block,
                                 const rtypes::timestep& t)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d0.events.nest_block_drop"),
      mc_timestep(t),
      m_block(block) {}

/*******************************************************************************
 * Controllers
 ******************************************************************************/
void nest_block_drop::visit(fccd0::dpo_controller& controller) {
  controller.ndc_pusht();

  visit(*controller.fsm());
  ER_INFO("Dropped block%d in nest", block()->id().v());

  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(fccd0::odpo_controller& controller) {
  controller.ndc_pusht();

  visit(*controller.fsm());
  ER_INFO("Dropped block%d in nest", block()->id().v());

  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(fccd0::mdpo_controller& controller) {
  controller.ndc_pusht();

  visit(*controller.fsm());
  ER_INFO("Dropped block%d in nest", block()->id().v());

  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(fccd0::omdpo_controller& controller) {
  controller.ndc_pusht();

  visit(*controller.fsm());
  ER_INFO("Dropped block%d in nest", block()->id().v());

  controller.ndc_pop();
} /* visit() */

/*******************************************************************************
 * FSMs
 ******************************************************************************/
void nest_block_drop::visit(fsm::d0::dpo_fsm& fsm) {
  fsm.inject_event(ffsm::foraging_signal::ekBLOCK_DROP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void nest_block_drop::visit(ffsm::d0::free_block_to_nest_fsm& fsm) {
  fsm.inject_event(ffsm::foraging_signal::ekBLOCK_DROP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

/*******************************************************************************
 * Tasks
 ******************************************************************************/
void nest_block_drop::visit(ftasks::d0::generalist& task) {
  visit(*static_cast<ffsm::d0::free_block_to_nest_fsm*>(task.mechanism()));
} /* visit() */

NS_END(events, d0, cognitive, controller, fordyca);
