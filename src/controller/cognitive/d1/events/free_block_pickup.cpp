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
#include "fordyca/controller/cognitive/d1/events/free_block_pickup.hpp"

#include "cosm/repr/sim_block3D.hpp"

#include "fordyca/controller/cognitive/d1/bitd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_omdpo_controller.hpp"
#include "fordyca/events/free_block_interactor.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/d0/free_block_to_nest_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/subsystem/perception/ds/dpo_semantic_map.hpp"
#include "fordyca/subsystem/perception/ds/dpo_store.hpp"
#include "fordyca/subsystem/perception/foraging_perception_subsystem.hpp"
#include "fordyca/tasks/d1/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d1, events);

using base_pickup = fccd0::events::free_block_pickup;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
free_block_pickup::free_block_pickup(crepr::sim_block3D* block,
                                     const rtypes::type_uuid& id,
                                     const rtypes::timestep& t)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d1.events.free_block_pickup"),
      base_pickup(block, id, t) {}

/*******************************************************************************
 * Controllers
 ******************************************************************************/
void free_block_pickup::visit(fccd1::bitd_dpo_controller& c) {
  c.ndc_uuid_push();

  controller_process(c, *c.perception()->template model<fspds::dpo_store>());
  ER_INFO("Picked up block%d", block()->id().v());

  c.ndc_uuid_pop();
} /* visit() */

void free_block_pickup::visit(fccd1::bitd_odpo_controller& c) {
  c.ndc_uuid_push();

  controller_process(c, *c.perception()->template model<fspds::dpo_store>());
  ER_INFO("Picked up block%d", block()->id().v());

  c.ndc_uuid_pop();
} /* visit() */

void free_block_pickup::visit(fccd1::bitd_mdpo_controller& c) {
  c.ndc_uuid_push();

  controller_process(c,
                     *c.perception()->template model<fspds::dpo_semantic_map>());
  ER_INFO("Picked up block%d", block()->id().v());

  c.ndc_uuid_pop();
} /* visit() */

void free_block_pickup::visit(fccd1::bitd_omdpo_controller& c) {
  c.ndc_uuid_push();

  controller_process(c,
                     *c.perception()->template model<fspds::dpo_semantic_map>());
  ER_INFO("Picked up block%d", block()->id().v());

  c.ndc_uuid_pop();
} /* visit() */

/*******************************************************************************
 * FSMs
 ******************************************************************************/
void free_block_pickup::visit(fsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_PICKUP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

NS_END(events, d1, cognitive, controller, fordyca);
