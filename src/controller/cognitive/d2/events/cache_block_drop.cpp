/**
 * \file cache_block_drop.cpp
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
#include "fordyca/controller/cognitive/d2/events/cache_block_drop.hpp"

#include "cosm/arena/repr/arena_cache.hpp"

#include "fordyca/controller/cognitive/cache_sel_matrix.hpp"
#include "fordyca/controller/cognitive/d2/birtd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_omdpo_controller.hpp"
#include "fordyca/subsystem/perception/mdpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/ds/dpo_semantic_map.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/d2/cache_transferer.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2, events);
using base_drop = fccd1::events::cache_block_drop;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_block_drop::cache_block_drop(
    std::unique_ptr<crepr::base_block3D> block,
    carepr::arena_cache* cache,
    const rtypes::discretize_ratio& resolution)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.events.d2.cache_block_drop"),
      base_drop(std::move(block), cache, resolution) {}

/*******************************************************************************
 * Controllers
 ******************************************************************************/
void cache_block_drop::visit(fccd2::birtd_dpo_controller& controller) {
  controller.ndc_push();

  if (dispatch_cache_interactor(controller.current_task(),
                                   controller.cache_sel_matrix())) {
    controller.csel_exception_added(true);
  }
  ER_INFO(
      "Dropped block%d in cache%d,task='%s'",
      block()->id().v(),
      cache()->id().v(),
      dynamic_cast<cta::logical_task*>(controller.current_task())->name().c_str());

  controller.ndc_pop();
} /* visit() */

void cache_block_drop::visit(fccd2::birtd_mdpo_controller& controller) {
  controller.ndc_push();

  if (dispatch_cache_interactor(controller.current_task(),
                                controller.cache_sel_matrix())) {
    controller.csel_exception_added(true);
  }
  base_drop::visit(*controller.perception()->model<fspds::dpo_semantic_map>());
  ER_INFO(
      "Dropped block%d in cache%d,task='%s'",
      block()->id().v(),
      cache()->id().v(),
      dynamic_cast<cta::logical_task*>(controller.current_task())->name().c_str());

  controller.ndc_pop();
} /* visit() */

void cache_block_drop::visit(fccd2::birtd_odpo_controller& controller) {
  controller.ndc_push();

  if (dispatch_cache_interactor(controller.current_task(),
                                   controller.cache_sel_matrix())) {
    controller.csel_exception_added(true);
  }
  ER_INFO(
      "Dropped block%d in cache%d,task='%s'",
      block()->id().v(),
      cache()->id().v(),
      dynamic_cast<cta::logical_task*>(controller.current_task())->name().c_str());

  controller.ndc_pop();
} /* visit() */

void cache_block_drop::visit(fccd2::birtd_omdpo_controller& controller) {
  controller.ndc_push();

  if (dispatch_cache_interactor(controller.current_task(),
                                   controller.cache_sel_matrix())) {
    controller.csel_exception_added(true);
  }
  base_drop::visit(*controller.perception()->model<fspds::dpo_semantic_map>());
  ER_INFO(
      "Dropped block%d in cache%d,task='%s'",
      block()->id().v(),
      cache()->id().v(),
      dynamic_cast<cta::logical_task*>(controller.current_task())->name().c_str());

  controller.ndc_pop();
} /* visit() */

void cache_block_drop::visit(tasks::d2::cache_transferer& task) {
  base_drop::visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool cache_block_drop::dispatch_cache_interactor(
    ftasks::base_foraging_task* task,
    fccognitive::cache_sel_matrix* csel_matrix) {
  auto* polled = dynamic_cast<cta::polled_task*>(task);
  auto* interactor = dynamic_cast<fevents::existing_cache_interactor*>(task);
  bool ret = false;
  ER_ASSERT(nullptr != interactor,
            "Non existing cache interactor task %s causing cached block drop",
            polled->name().c_str());

  if (tasks::d2::foraging_task::kCacheTransfererName == polled->name()) {
    ER_INFO("Added cache%d@%s/%s to pickup exception list,task='%s'",
            cache()->id().v(),
            rcppsw::to_string(cache()->rcenter2D()).c_str(),
            rcppsw::to_string(cache()->dcenter2D()).c_str(),
            polled->name().c_str());
    csel_matrix->sel_exception_add(
        { cache()->id(), fccognitive::cache_sel_exception::ekPICKUP });
    ret = true;
  }
  interactor->accept(*this);
  return ret;
} /* dispatch_d2_cache_interactor() */

NS_END(events, d2, cognitive, controller, fordyca);
