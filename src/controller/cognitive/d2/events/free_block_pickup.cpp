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
#include "fordyca/controller/cognitive/d2/events/free_block_pickup.hpp"

#include "fordyca/controller/cognitive/d2/birtd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_omdpo_controller.hpp"
#include "fordyca/subsystem/perception/dpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/mdpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/ds/dpo_semantic_map.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/d2/cache_finisher.hpp"
#include "fordyca/tasks/d2/cache_starter.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2, events);

using base_pickup = fccd1::events::free_block_pickup;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
free_block_pickup::free_block_pickup(crepr::sim_block3D* block,
                                     const rtypes::type_uuid& id,
                                     const rtypes::timestep& t)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d2.events.free_block_pickup"),
      base_pickup(block, id, t) {}

/*******************************************************************************
 * Controllers
 ******************************************************************************/
void free_block_pickup::visit(fccd2::birtd_dpo_controller& c) {
  c.ndc_uuid_push();

  controller_visit_impl(c, *c.perception()->template model<fspds::dpo_store>());
  ER_INFO("Picked up block%d", block()->id().v());

  c.ndc_uuid_pop();
} /* visit() */

void free_block_pickup::visit(fccd2::birtd_mdpo_controller& c) {
  c.ndc_uuid_push();

  controller_visit_impl(c, *c.perception()->template model<fspds::dpo_store>());
  ER_INFO("Picked up block%d", block()->id().v());

  c.ndc_uuid_pop();
} /* visit() */

void free_block_pickup::visit(fccd2::birtd_odpo_controller& c) {
  c.ndc_uuid_push();

  controller_visit_impl(c, *c.perception()->template model<fspds::dpo_semantic_map>());
  ER_INFO("Picked up block%d", block()->id().v());

  c.ndc_uuid_pop();
} /* visit() */

void free_block_pickup::visit(fccd2::birtd_omdpo_controller& c) {
  c.ndc_uuid_push();

  controller_visit_impl(c, *c.perception()->template model<fspds::dpo_semantic_map>());
  ER_INFO("Picked up block%d", block()->id().v());

  c.ndc_uuid_pop();
} /* visit() */

/*******************************************************************************
 * FSMs
 ******************************************************************************/
void free_block_pickup::visit(tasks::d2::cache_starter& task) {
  base_pickup::visit(*static_cast<ffsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

void free_block_pickup::visit(tasks::d2::cache_finisher& task) {
  base_pickup::visit(*static_cast<ffsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

NS_END(events, d2, cognitive, controller, fordyca);
