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
#include "fordyca/controller/cognitive/d2/events/cache_vanished.hpp"

#include "fordyca/controller/cognitive/d2/birtd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_omdpo_controller.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2, events);

using base_vanished = fccd1::events::cache_vanished;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_vanished::cache_vanished(const rtypes::type_uuid& cache_id)
    : ER_CLIENT_INIT("fordyca.events.cache_vanished"), base_vanished(cache_id) {}

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void cache_vanished::visit(fccd2::birtd_dpo_controller& controller) {
  controller.ndc_uuid_push();

  ER_INFO("Abort pickup/drop from/in cache: cache%d vanished", cache_id().v());
  dispatch_cache_interactor(controller.current_task());

  controller.ndc_uuid_pop();
} /* visit() */

void cache_vanished::visit(fccd2::birtd_mdpo_controller& controller) {
  controller.ndc_uuid_push();

  ER_INFO("Abort pickup/drop from/in cache: cache%d vanished", cache_id().v());
  dispatch_cache_interactor(controller.current_task());

  controller.ndc_uuid_pop();
} /* visit() */

void cache_vanished::visit(fccd2::birtd_odpo_controller& controller) {
  controller.ndc_uuid_push();

  ER_INFO("Abort pickup/drop from/in cache: cache%d vanished", cache_id().v());
  dispatch_cache_interactor(controller.current_task());

  controller.ndc_uuid_pop();
} /* visit() */

void cache_vanished::visit(fccd2::birtd_omdpo_controller& controller) {
  controller.ndc_uuid_push();

  ER_INFO("Abort pickup/drop from/in cache: cache%d vanished", cache_id().v());
  dispatch_cache_interactor(controller.current_task());

  controller.ndc_uuid_pop();
} /* visit() */

NS_END(events, d2, cognitive, controller, fordyca);
