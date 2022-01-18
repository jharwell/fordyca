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
#include "fordyca/controller/cognitive/d2/events/block_vanished.hpp"

#include "fordyca/controller/cognitive/d2/birtd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_omdpo_controller.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/d2/cache_finisher.hpp"
#include "fordyca/tasks/d2/cache_starter.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_vanished::block_vanished(const rtypes::type_uuid& block_id)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d2.events.block_vanished"),
      fccd1::events::block_vanished(block_id) {}

/*******************************************************************************
 * Controllers
 ******************************************************************************/
void block_vanished::visit(fccd2::birtd_mdpo_controller& controller) {
  controller.ndc_push();

  dispatch_free_block_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void block_vanished::visit(fccd2::birtd_dpo_controller& controller) {
  controller.ndc_push();

  dispatch_free_block_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void block_vanished::visit(fccd2::birtd_omdpo_controller& controller) {
  controller.ndc_push();

  dispatch_free_block_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void block_vanished::visit(fccd2::birtd_odpo_controller& controller) {
  controller.ndc_push();

  dispatch_free_block_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

/*******************************************************************************
 * Tasks
 ******************************************************************************/
void block_vanished::visit(ftasks::d2::cache_starter& task) {
  visit(*static_cast<ffsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

void block_vanished::visit(ftasks::d2::cache_finisher& task) {
  visit(*static_cast<ffsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

/*******************************************************************************
 * FSMs
 ******************************************************************************/
void block_vanished::visit(ffsm::block_to_goal_fsm& fsm) {
  fccd1::events::block_vanished::visit(fsm);
} /* visit() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_vanished::dispatch_free_block_interactor(
    ftasks::base_foraging_task* const task) {
  ER_INFO("Abort pickup while executing task %s: block%d vanished",
          dynamic_cast<cta::logical_task*>(task)->name().c_str(),
          block_id().v());
  auto* interactor = dynamic_cast<fevents::free_block_interactor*>(task);
  ER_ASSERT(nullptr != interactor,
            "Non-free block interactor task %s triggered block vanished event",
            dynamic_cast<cta::logical_task*>(task)->name().c_str());
  interactor->accept(*this);
} /* dispatch_free_block_interactor() */

NS_END(events, d2, cognitive, controller, fordyca);
