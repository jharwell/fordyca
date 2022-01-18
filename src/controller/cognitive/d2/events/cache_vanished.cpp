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
#include "fordyca/controller/cognitive/d2/events/cache_vanished.hpp"

#include "fordyca/controller/cognitive/d2/birtd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_omdpo_controller.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/d2/cache_transferer.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2, events);

using base_vanished = fccd1::events::cache_vanished;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_vanished::cache_vanished(const rtypes::type_uuid& cache_id)
    : ER_CLIENT_INIT("fordyca.events.cache_vanished"),
      base_vanished(cache_id) {}

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

/*******************************************************************************
 * Tasks
 ******************************************************************************/
void cache_vanished::visit(tasks::d2::cache_transferer& task) {
  base_vanished::visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

NS_END(events, d2, cognitive, controller, fordyca);
