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
#include "fordyca/controller/cognitive/d0/events/free_block_pickup.hpp"

#include "cosm/repr/base_block3D.hpp"

#include "fordyca/controller/cognitive/d0/dpo_controller.hpp"
#include "fordyca/controller/cognitive/d0/mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d0/odpo_controller.hpp"
#include "fordyca/controller/cognitive/d0/omdpo_controller.hpp"
#include "fordyca/subsystem/perception/dpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/mdpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/ds/dpo_semantic_map.hpp"
#include "fordyca/events/cell2D_empty.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
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
free_block_pickup::free_block_pickup(crepr::base_block3D* block,
                                     const rtypes::type_uuid& id,
                                     const rtypes::timestep& t)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d0.events.free_block_pickup"),
      ccops::base_block_pickup(block, id, t) {}


/*******************************************************************************
 * Controllers
 ******************************************************************************/
void free_block_pickup::visit(fccd0::mdpo_controller& c) {
  c.ndc_uuid_push();

  controller_visit_impl(c, *c.perception()->template model<fspds::dpo_semantic_map>());
  ER_INFO("Picked up block%d", block()->id().v());

  c.ndc_uuid_pop();
} /* visit() */

void free_block_pickup::visit(fccd0::omdpo_controller& c) {
  c.ndc_uuid_push();

  controller_visit_impl(c, *c.perception()->template model<fspds::dpo_semantic_map>());
  ER_INFO("Picked up block%d", block()->id().v());

  c.ndc_uuid_pop();
} /* visit() */

void free_block_pickup::visit(fccd0::dpo_controller& c) {
  c.ndc_uuid_push();

  controller_visit_impl(c, *c.perception()->template model<fspds::dpo_store>());
  ER_INFO("Picked up block%d", block()->id().v());

  c.ndc_uuid_pop();
} /* visit() */

void free_block_pickup::visit(fccd0::odpo_controller& c) {
  c.ndc_uuid_push();

  controller_visit_impl(c, *c.perception()->template model<fspds::dpo_store>());
  ER_INFO("Picked up block%d", block()->id().v());

  c.ndc_uuid_pop();
} /* visit() */

/*******************************************************************************
 * Tasks
 ******************************************************************************/
void free_block_pickup::visit(ftasks::d0::generalist& task) {
  visit(*static_cast<ffsm::d0::free_block_to_nest_fsm*>(task.mechanism()));
} /* visit() */

/*******************************************************************************
 * FSMs
 ******************************************************************************/
void free_block_pickup::visit(ffsm::d0::dpo_fsm& fsm) {
  fsm.inject_event(ffsm::foraging_signal::ekBLOCK_PICKUP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void free_block_pickup::visit(ffsm::d0::free_block_to_nest_fsm& fsm) {
  fsm.inject_event(ffsm::foraging_signal::ekBLOCK_PICKUP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

/*******************************************************************************
 * Data Structures
 ******************************************************************************/
void free_block_pickup::visit(fspds::dpo_store& store) {
  ER_ASSERT(store.contains(block()),
            "Block%d not in DPO store",
            block()->id().v());
  store.block_remove(block());
  ER_ASSERT(!store.contains(block()),
            "Block%d in DPO store after removal",
            block()->id().v());
} /* visit() */

void free_block_pickup::visit(fspds::dpo_semantic_map& map) {
  auto& cell = map.access<fspds::occupancy_grid::kCell>(coord());

  ER_ASSERT(block()->danchor2D() == cell.loc(),
            "Coordinates for block%d@%s/cell@%s do not agree",
            block()->id().v(),
            rcppsw::to_string(block()->danchor2D()).c_str(),
            rcppsw::to_string(cell.loc()).c_str());

  ER_ASSERT(block()->id() == cell.block3D()->id(),
            "Pickup/cell block mismatch: %d vs %d",
            block()->id().v(),
            cell.block3D()->id().v());
  /*
   * @bug: This should just be an assert. However, due to FORDYCA#242, the fact
   * that blocks can appear close to the wall, and FORDYCA#82, this may not
   * always be true (and the fact that it isn't is not an indication of
   * inconsistent simulation state :-( ). This can happen if, for example, a
   * robot is exploring for a block very near the edge of the arena, and happens
   * to drive over a block. In that case the block is not in its LOS (BUG!), or
   * in its occupancy grid, and hence the assertion failure here.
   */
  /* ER_ASSERT(cell.state_has_block(), "cell does not contain block"); */
  if (cell.state_has_block()) {
    map.block_remove(cell.block3D());
  }
} /* visit() */

NS_END(events, d0, cognitive, controller, fordyca);
