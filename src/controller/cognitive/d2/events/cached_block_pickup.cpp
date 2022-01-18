/**
 * \file cached_block_pickup.cpp
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
#include "fordyca/controller/cognitive/d2/events/cached_block_pickup.hpp"

#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/repr/base_block3D.hpp"

#include "fordyca/controller/cognitive/cache_sel_matrix.hpp"

#include "fordyca/controller/cognitive/d2/birtd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_omdpo_controller.hpp"
#include "fordyca/subsystem/perception/dpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/mdpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/ds/dpo_semantic_map.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/d2/foraging_task.hpp"
#include "fordyca/tasks/d2/cache_collector.hpp"
#include "fordyca/tasks/d2/cache_transferer.hpp"
#include "fordyca/fsm/d1/cached_block_to_nest_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2, events);

using carepr::base_cache;
using base_pickup = fccd1::events::cached_block_pickup;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cached_block_pickup::cached_block_pickup(const carepr::arena_cache* cache,
                                         crepr::base_block3D* block,
                                         const rtypes::type_uuid& id,
                                         const rtypes::timestep& t)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d2.events.cached_block_pickup"),
      base_pickup(cache, block, id, t) {}

cached_block_pickup::~cached_block_pickup(void) = default;

/*******************************************************************************
 * Controllers
 ******************************************************************************/
void cached_block_pickup::visit(fccd2::birtd_dpo_controller& controller) {
  controller.ndc_push();

  base_pickup::visit(*controller.perception()->model<fspds::dpo_semantic_map>());
  base_pickup::visit(static_cast<ccontroller::block_carrying_controller&>(controller));

  if (dispatch_cache_interactor(controller.current_task(),
                                   controller.cache_sel_matrix())) {
    controller.csel_exception_added(true);
  }

  controller.ndc_pop();
} /* visit() */

void cached_block_pickup::visit(fccd2::birtd_mdpo_controller& controller) {
  controller.ndc_push();

  base_pickup::visit(*controller.perception()->model<fspds::dpo_semantic_map>());
  base_pickup::visit(static_cast<ccontroller::block_carrying_controller&>(controller));

  if (dispatch_cache_interactor(controller.current_task(),
                                   controller.cache_sel_matrix())) {
    controller.csel_exception_added(true);
  }
  controller.ndc_pop();
} /* visit() */

void cached_block_pickup::visit(fccd2::birtd_odpo_controller& controller) {
  controller.ndc_push();

  base_pickup::visit(*controller.perception()->model<fspds::dpo_store>());
  base_pickup::visit(static_cast<ccontroller::block_carrying_controller&>(controller));

  if (dispatch_cache_interactor(controller.current_task(),
                                   controller.cache_sel_matrix())) {
    controller.csel_exception_added(true);
  }

  controller.ndc_pop();
} /* visit() */

void cached_block_pickup::visit(fccd2::birtd_omdpo_controller& controller) {
  controller.ndc_push();

  base_pickup::visit(*controller.perception()->model<fspds::dpo_semantic_map>());
  base_pickup::visit(static_cast<ccontroller::block_carrying_controller&>(controller));

  if (dispatch_cache_interactor(controller.current_task(),
                                   controller.cache_sel_matrix())) {
    controller.csel_exception_added(true);
  }
  controller.ndc_pop();
} /* visit() */

/*******************************************************************************
 * Tasks
 ******************************************************************************/
void cached_block_pickup::visit(tasks::d2::cache_transferer& task) {
  base_pickup::visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */
void cached_block_pickup::visit(tasks::d2::cache_collector& task) {
  base_pickup::visit(*static_cast<ffsm::d1::cached_block_to_nest_fsm*>(task.mechanism()));
} /* visit() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool cached_block_pickup::dispatch_cache_interactor(
    ftasks::base_foraging_task* task,
    fccognitive::cache_sel_matrix* csel_matrix) {
  auto* polled = dynamic_cast<cta::polled_task*>(task);
  auto* interactor = dynamic_cast<fevents::existing_cache_interactor*>(task);
  bool ret = false;

  ER_ASSERT(nullptr != interactor,
            "Non existing cache interactor task %s causing cached block pickup",
            polled->name().c_str());

  if (ftasks::d2::foraging_task::kCacheTransfererName == polled->name()) {
    ER_INFO("Added cache%d@%s/%s to drop exception list,task='%s'",
            cache()->id().v(),
            rcppsw::to_string(cache()->rcenter2D()).c_str(),
            rcppsw::to_string(cache()->dcenter2D()).c_str(),
            polled->name().c_str());
    csel_matrix->sel_exception_add(
        { cache()->id(), fccognitive::cache_sel_exception::ekDROP });
    ret = true;
  }
  interactor->accept(*this);
  ER_INFO("Picked up block%d from cache%d,task='%s'",
          block()->id().v(),
          cache()->id().v(),
          polled->name().c_str());
  return ret;
} /* dispatch_cache_interactor() */

NS_END(events, d2, cognitive, controller, fordyca);
