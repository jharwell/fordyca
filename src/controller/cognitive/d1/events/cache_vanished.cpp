/**
 * \file cache_vanished.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/cognitive/d1/events/cache_vanished.hpp"

#include "fordyca/controller/cognitive/d1/bitd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_omdpo_controller.hpp"
#include "fordyca/events/existing_cache_interactor.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/d1/cached_block_to_nest_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/d1/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d1, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_vanished::cache_vanished(const rtypes::type_uuid& cache_id)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d1events.cache_vanished"),
      mc_cache_id(cache_id) {}

/*******************************************************************************
 * Controllers
 ******************************************************************************/
void cache_vanished::visit(fccd1::bitd_dpo_controller& controller) {
  controller.ndc_uuid_push();

  dispatch_cache_interactor(controller.current_task());

  controller.ndc_uuid_pop();
} /* visit() */

void cache_vanished::visit(fccd1::bitd_mdpo_controller& controller) {
  controller.ndc_uuid_push();

  dispatch_cache_interactor(controller.current_task());

  controller.ndc_uuid_pop();
} /* visit() */

void cache_vanished::visit(fccd1::bitd_odpo_controller& controller) {
  controller.ndc_uuid_push();

  dispatch_cache_interactor(controller.current_task());

  controller.ndc_uuid_pop();
} /* visit() */

void cache_vanished::visit(fccd1::bitd_omdpo_controller& controller) {
  controller.ndc_uuid_push();

  dispatch_cache_interactor(controller.current_task());

  controller.ndc_uuid_pop();
} /* visit() */

/*******************************************************************************
 * FSMs
 ******************************************************************************/
void cache_vanished::visit(ffsm::d1::cached_block_to_nest_fsm& fsm) {
  fsm.inject_event(ffsm::foraging_signal::ekCACHE_VANISHED,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void cache_vanished::visit(ffsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(ffsm::foraging_signal::ekCACHE_VANISHED,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_vanished::dispatch_cache_interactor(
    tasks::base_foraging_task* const task) {
  ER_INFO("Abort pickup/drop from/in cache: cache%d vanished", mc_cache_id.v());

  auto* interactor = dynamic_cast<fevents::existing_cache_interactor*>(task);
  ER_ASSERT(nullptr != interactor,
            "Non existing cache interactor task %s received cache vanished event",
            dynamic_cast<cta::logical_task*>(task)->name().c_str());
  interactor->accept(*this);
} /* dispatch_cache_interactor() */

NS_END(events, d1, cognitive, controller, fordyca);
