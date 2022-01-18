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
#include "fordyca/controller/cognitive/d2/events/nest_block_drop.hpp"

#include "fordyca/controller/cognitive/d2/birtd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_omdpo_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
nest_block_drop::nest_block_drop(crepr::base_block3D* block,
                                 const rtypes::timestep& t)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d2.events.nest_block_drop"),
      fccd1::events::nest_block_drop(block, t) {}

/*******************************************************************************
 * Controllers
 ******************************************************************************/
void nest_block_drop::visit(fccd2::birtd_mdpo_controller& controller) {
  controller.ndc_push();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", block()->id().v());

  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(fccd2::birtd_dpo_controller& controller) {
  controller.ndc_push();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", block()->id().v());

  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(fccd2::birtd_odpo_controller& controller) {
  controller.ndc_push();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", block()->id().v());

  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(fccd2::birtd_omdpo_controller& controller) {
  controller.ndc_push();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", block()->id().v());

  controller.ndc_pop();
} /* visit() */

NS_END(events, d2, cognitive, controller, fordyca);
