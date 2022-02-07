/**
 * \file free_block_pickup.cpp
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
#include "fordyca/controller/cognitive/d1/events/free_block_pickup.hpp"

#include "cosm/repr/sim_block3D.hpp"

#include "fordyca/controller/cognitive/d1/bitd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_omdpo_controller.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/d1/harvester.hpp"
#include "fordyca/subsystem/perception/foraging_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/ds/dpo_store.hpp"
#include "fordyca/subsystem/perception/ds/dpo_semantic_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d1, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
free_block_pickup::free_block_pickup(crepr::sim_block3D* block,
                                     const rtypes::type_uuid& id,
                                     const rtypes::timestep& t)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d1.events.free_block_pickup"),
      fccd0::events::free_block_pickup(block, id, t) {}


/*******************************************************************************
 * Controllers
 ******************************************************************************/
void free_block_pickup::visit(fccd1::bitd_dpo_controller& c) {
  c.ndc_uuid_push();

  controller_visit_impl(c, *c.perception()->template model<fspds::dpo_store>());
  ER_INFO("Picked up block%d", block()->id().v());

  c.ndc_uuid_pop();
} /* visit() */

void free_block_pickup::visit(fccd1::bitd_odpo_controller& c) {
  c.ndc_uuid_push();

  controller_visit_impl(c, *c.perception()->template model<fspds::dpo_store>());
  ER_INFO("Picked up block%d", block()->id().v());

  c.ndc_uuid_pop();
} /* visit() */

void free_block_pickup::visit(fccd1::bitd_mdpo_controller& c) {
  c.ndc_uuid_push();

  controller_visit_impl(c, *c.perception()->template model<fspds::dpo_semantic_map>());
  ER_INFO("Picked up block%d", block()->id().v());

  c.ndc_uuid_pop();
} /* visit() */

void free_block_pickup::visit(fccd1::bitd_omdpo_controller& c) {
  c.ndc_uuid_push();

  controller_visit_impl(c, *c.perception()->template model<fspds::dpo_semantic_map>());
  ER_INFO("Picked up block%d", block()->id().v());

  c.ndc_uuid_pop();
} /* visit() */

/*******************************************************************************
 * Tasks
 ******************************************************************************/
void free_block_pickup::visit(tasks::d1::harvester& task) {
  visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

/*******************************************************************************
 * FSMs
 ******************************************************************************/
void free_block_pickup::visit(fsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_PICKUP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void free_block_pickup::dispatch_free_block_interactor(
    tasks::base_foraging_task* const task) {
  auto* polled RCPPSW_UNUSED = dynamic_cast<cta::polled_task*>(task);
  auto* interactor = dynamic_cast<fevents::free_block_interactor*>(task);
  ER_ASSERT(nullptr != interactor,
            "Non free block interactor task %s causing free block pickup",
            polled->name().c_str());
  interactor->accept(*this);
} /* dispatch_free_block_interactor() */

NS_END(events, d1, cognitive, controller, fordyca);
