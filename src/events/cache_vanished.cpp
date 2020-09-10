/**
 * \file cache_vanished.cpp
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
#include "fordyca/events/cache_vanished.hpp"

#include "fordyca/controller/cognitive/depth1/bitd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/depth1/bitd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/depth1/bitd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/depth1/bitd_omdpo_controller.hpp"
#include "fordyca/controller/cognitive/depth2/birtd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/depth2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/depth2/birtd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/depth2/birtd_omdpo_controller.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/depth1/collector.hpp"
#include "fordyca/tasks/depth1/harvester.hpp"
#include "fordyca/tasks/depth2/cache_transferer.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_vanished::cache_vanished(const rtypes::type_uuid& cache_id)
    : ER_CLIENT_INIT("fordyca.events.cache_vanished"), mc_cache_id(cache_id) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_vanished::dispatch_cache_interactor(
    tasks::base_foraging_task* const task) {
  ER_INFO("Abort pickup/drop from/in cache: cache%d vanished", mc_cache_id.v());

  auto* interactor = dynamic_cast<events::existing_cache_interactor*>(task);
  ER_ASSERT(
      nullptr != interactor,
      "Non existing cache interactor task %s received cache vanished event",
      dynamic_cast<cta::logical_task*>(task)->name().c_str());
  interactor->accept(*this);
} /* dispatch_cache_interactor() */

/*******************************************************************************
 * Depth1 Foraging
 ******************************************************************************/
void cache_vanished::visit(controller::cognitive::depth1::bitd_dpo_controller& controller) {
  controller.ndc_pusht();

  dispatch_cache_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void cache_vanished::visit(controller::cognitive::depth1::bitd_mdpo_controller& controller) {
  controller.ndc_pusht();

  dispatch_cache_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void cache_vanished::visit(controller::cognitive::depth1::bitd_odpo_controller& controller) {
  controller.ndc_pusht();

  dispatch_cache_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void cache_vanished::visit(controller::cognitive::depth1::bitd_omdpo_controller& controller) {
  controller.ndc_pusht();

  dispatch_cache_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void cache_vanished::visit(tasks::depth1::collector& task) {
  visit(*static_cast<fsm::depth1::cached_block_to_nest_fsm*>(task.mechanism()));
} /* visit() */

void cache_vanished::visit(tasks::depth1::harvester& task) {
  visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

void cache_vanished::visit(fsm::depth1::cached_block_to_nest_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekCACHE_VANISHED,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void cache_vanished::visit(fsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekCACHE_VANISHED,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void cache_vanished::visit(controller::cognitive::depth2::birtd_dpo_controller& controller) {
  controller.ndc_pusht();

  dispatch_cache_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void cache_vanished::visit(controller::cognitive::depth2::birtd_mdpo_controller& controller) {
  controller.ndc_pusht();

  dispatch_cache_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void cache_vanished::visit(controller::cognitive::depth2::birtd_odpo_controller& controller) {
  controller.ndc_pusht();

  dispatch_cache_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void cache_vanished::visit(
    controller::cognitive::depth2::birtd_omdpo_controller& controller) {
  controller.ndc_pusht();

  dispatch_cache_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void cache_vanished::visit(tasks::depth2::cache_transferer& task) {
  visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

NS_END(detail, events, fordyca);
