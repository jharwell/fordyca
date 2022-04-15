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
#include "fordyca/controller/cognitive/d1/events/cache_block_drop.hpp"

#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/repr/base_block3D.hpp"

#include "fordyca/controller/cognitive/cache_sel_matrix.hpp"
#include "fordyca/controller/cognitive/d1/bitd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_omdpo_controller.hpp"
#include "fordyca/events/existing_cache_interactor.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/subsystem/perception/ds/dpo_semantic_map.hpp"
#include "fordyca/subsystem/perception/mdpo_perception_subsystem.hpp"
#include "fordyca/tasks/d1/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d1, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_block_drop::cache_block_drop(std::unique_ptr<crepr::base_block3D> block,
                                   carepr::arena_cache* cache,
                                   const rtypes::discretize_ratio& resolution)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.events.d1.cache_block_drop"),
      cell2D_op(cache->dcenter2D()),
      mc_resolution(resolution),
      m_block(std::move(block)),
      m_cache(cache) {}

/*******************************************************************************
 * Controllers
 ******************************************************************************/
void cache_block_drop::visit(
    controller::cognitive::d1::bitd_dpo_controller& controller) {
  controller.ndc_uuid_push();

  dispatch_cache_interactor(controller.current_task());

  ER_INFO(
      "Dropped block%d in cache%d,task='%s'",
      m_block->id().v(),
      m_cache->id().v(),
      dynamic_cast<cta::logical_task*>(controller.current_task())->name().c_str());

  controller.ndc_uuid_pop();
} /* visit() */

void cache_block_drop::visit(
    controller::cognitive::d1::bitd_mdpo_controller& controller) {
  controller.ndc_uuid_push();

  visit(*controller.perception()->model<fspds::dpo_semantic_map>());
  dispatch_cache_interactor(controller.current_task());

  ER_INFO(
      "Dropped block%d in cache%d,task='%s'",
      m_block->id().v(),
      m_cache->id().v(),
      dynamic_cast<cta::logical_task*>(controller.current_task())->name().c_str());

  controller.ndc_uuid_pop();
} /* visit() */

void cache_block_drop::visit(
    controller::cognitive::d1::bitd_odpo_controller& controller) {
  controller.ndc_uuid_push();

  dispatch_cache_interactor(controller.current_task());

  ER_INFO(
      "Dropped block%d in cache%d,task='%s'",
      m_block->id().v(),
      m_cache->id().v(),
      dynamic_cast<cta::logical_task*>(controller.current_task())->name().c_str());

  controller.ndc_uuid_pop();
} /* visit() */

void cache_block_drop::visit(
    controller::cognitive::d1::bitd_omdpo_controller& controller) {
  controller.ndc_uuid_push();

  visit(*controller.perception()->model<fspds::dpo_semantic_map>());
  dispatch_cache_interactor(controller.current_task());

  ER_INFO(
      "Dropped block%d in cache%d,task='%s'",
      m_block->id().v(),
      m_cache->id().v(),
      dynamic_cast<cta::logical_task*>(controller.current_task())->name().c_str());

  controller.ndc_uuid_pop();
} /* visit() */

/*******************************************************************************
 * FSMs
 ******************************************************************************/
void cache_block_drop::visit(cfsm::cell2D_fsm& fsm) {
  ER_ASSERT(fsm.state_has_cache(), "Cell does not contain a cache");
  fsm.event_block_drop();
} /* visit() */

void cache_block_drop::visit(fsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_DROP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

/*******************************************************************************
 * Data Structures
 ******************************************************************************/
void cache_block_drop::visit(fspds::dpo_semantic_map& map) {
  visit(map.access<fspds::occupancy_grid::kCell>(x(), y()));
} /* visit() */

void cache_block_drop::visit(cds::cell2D& cell) {
  ER_ASSERT(0 != cell.loc().x() && 0 != cell.loc().y(),
            "Cell does not have coordinates");

  visit(cell.fsm());
  ER_ASSERT(m_cache->n_blocks() == cell.block_count(),
            "Cache/cell disagree on # of blocks: cache=%zu/cell=%zu",
            m_cache->n_blocks(),
            cell.block_count());
} /* visit() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_block_drop::dispatch_cache_interactor(
    tasks::base_foraging_task* const task) {
  auto* interactor = dynamic_cast<fevents::existing_cache_interactor*>(task);

  ER_ASSERT(nullptr != interactor,
            "Non existing cache interactor task %s causing cached block drop",
            dynamic_cast<cta::logical_task*>(task)->name().c_str());
  interactor->accept(*this);
} /* dispatch_cache_interactor() */

NS_END(events, d1, cognitive, controller, fordyca);
