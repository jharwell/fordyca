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
  controller.ndc_uuid_push();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", block()->id().v());

  controller.ndc_uuid_pop();
} /* visit() */

void nest_block_drop::visit(fccd2::birtd_dpo_controller& controller) {
  controller.ndc_uuid_push();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", block()->id().v());

  controller.ndc_uuid_pop();
} /* visit() */

void nest_block_drop::visit(fccd2::birtd_odpo_controller& controller) {
  controller.ndc_uuid_push();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", block()->id().v());

  controller.ndc_uuid_pop();
} /* visit() */

void nest_block_drop::visit(fccd2::birtd_omdpo_controller& controller) {
  controller.ndc_uuid_push();

  dispatch_nest_interactor(controller.current_task());
  ER_INFO("Dropped block%d in nest", block()->id().v());

  controller.ndc_uuid_pop();
} /* visit() */

NS_END(events, d2, cognitive, controller, fordyca);
