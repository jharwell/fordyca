/**
 * @file cache_block_drop.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/events/cache_block_drop.hpp"
#include "fordyca/controller/cache_sel_matrix.hpp"
#include "fordyca/controller/depth1/gp_dpo_controller.hpp"
#include "fordyca/controller/depth1/gp_mdpo_controller.hpp"
#include "fordyca/controller/depth1/gp_odpo_controller.hpp"
#include "fordyca/controller/depth1/gp_omdpo_controller.hpp"
#include "fordyca/controller/depth2/grp_dpo_controller.hpp"
#include "fordyca/controller/depth2/grp_mdpo_controller.hpp"
#include "fordyca/controller/depth2/grp_odpo_controller.hpp"
#include "fordyca/controller/depth2/grp_omdpo_controller.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/ds/arena_map.hpp"
#include "fordyca/ds/cell2D.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/repr/arena_cache.hpp"
#include "fordyca/repr/base_block.hpp"
#include "fordyca/tasks/depth1/foraging_task.hpp"
#include "fordyca/tasks/depth1/harvester.hpp"
#include "fordyca/tasks/depth2/cache_transferer.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);
using ds::arena_grid;
using ds::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_block_drop::cache_block_drop(
    const std::shared_ptr<repr::base_block>& block,
    const std::shared_ptr<repr::arena_cache>& cache,
    rtypes::discretize_ratio resolution)
    : ER_CLIENT_INIT("fordyca.events.cache_block_drop"),
      cell_op(cache->dloc()),
      mc_resolution(resolution),
      m_block(block),
      m_cache(cache) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_block_drop::dispatch_d1_cache_interactor(
    tasks::base_foraging_task* const task) {
  auto* interactor = dynamic_cast<events::existing_cache_interactor*>(task);

  ER_ASSERT(nullptr != interactor,
            "Non existing cache interactor task %s causing cached block drop",
            dynamic_cast<rta::logical_task*>(task)->name().c_str());
  interactor->accept(*this);
} /* dispatch_d1_cache_interactor() */

bool cache_block_drop::dispatch_d2_cache_interactor(
    tasks::base_foraging_task* task,
    controller::cache_sel_matrix* csel_matrix) {
  auto* polled = dynamic_cast<rta::polled_task*>(task);
  auto* interactor = dynamic_cast<events::existing_cache_interactor*>(task);
  bool ret = false;
  ER_ASSERT(nullptr != interactor,
            "Non existing cache interactor task %s causing cached block drop",
            polled->name().c_str());

  if (tasks::depth2::foraging_task::kCacheTransfererName == polled->name()) {
    ER_INFO("Added cache%d@%s to pickup exception list,task='%s'",
            m_block->id(),
            m_block->rloc().to_str().c_str(),
            polled->name().c_str());
    csel_matrix->sel_exception_add(
        {m_cache->id(), controller::cache_sel_exception::kPickup});
    ret = true;
  }
  interactor->accept(*this);
  return ret;
} /* dispatch_d2_cache_interactor() */

/*******************************************************************************
 * Depth1 Foraging
 ******************************************************************************/
void cache_block_drop::visit(ds::cell2D& cell) {
  ER_ASSERT(0 != cell.loc().x() && 0 != cell.loc().y(),
            "Cell does not have coordinates");

  visit(cell.fsm());
  ER_ASSERT(m_cache->n_blocks() == cell.block_count(),
            "Cache/cell disagree on # of blocks: cache=%zu/cell=%zu",
            m_cache->n_blocks(),
            cell.block_count());
} /* visit() */

void cache_block_drop::visit(fsm::cell2D_fsm& fsm) {
  ER_ASSERT(fsm.state_has_cache(), "Cell does not contain a cache");
  fsm.event_block_drop();
} /* visit() */

void cache_block_drop::visit(ds::arena_map& map) {
  ER_ASSERT(-1 != m_block->robot_id(),
            "Undefined robot index for block%d",
            m_block->id());
  __rcsw_unused int index = m_block->robot_id();
  visit(*m_block);
  visit(*m_cache);
  visit(map.access<arena_grid::kCell>(cell_op::x(), cell_op::y()));
  ER_INFO("arena_map: fb%d dropped block%d in cache%d,total=[%s] (%zu)",
          index,
          m_block->id(),
          m_cache->id(),
          rcppsw::to_string(m_cache->blocks()).c_str(),
          m_cache->n_blocks());
} /* visit() */

void cache_block_drop::visit(ds::dpo_semantic_map& map) {
  visit(map.access<occupancy_grid::kCell>(x(), y()));
} /* visit() */

void cache_block_drop::visit(repr::base_block& block) {
  events::free_block_drop_visitor e(m_block, /* OK because we only have 1 block */
                                    rmath::vector2u(cell_op::x(), cell_op::y()),
                                    mc_resolution);
  e.visit(block);
} /* visit() */

void cache_block_drop::visit(repr::arena_cache& cache) {
  cache.block_add(m_block);
  cache.has_block_drop();
} /* visit() */

void cache_block_drop::visit(controller::depth1::gp_dpo_controller& controller) {
  controller.ndc_push();

  dispatch_d1_cache_interactor(controller.current_task());
  controller.block(nullptr);
  controller.block_manip_collator()->cache_drop_event(true);

  ER_INFO("Dropped block%d in cache%d,task='%s'",
          m_block->id(),
          m_cache->id(),
          dynamic_cast<rta::logical_task*>(controller.current_task())
              ->name()
              .c_str());

  controller.ndc_pop();
} /* visit() */

void cache_block_drop::visit(controller::depth1::gp_mdpo_controller& controller) {
  controller.ndc_push();

  visit(*controller.mdpo_perception()->map());
  dispatch_d1_cache_interactor(controller.current_task());
  controller.block(nullptr);
  controller.block_manip_collator()->cache_drop_event(true);

  ER_INFO("Dropped block%d in cache%d,task='%s'",
          m_block->id(),
          m_cache->id(),
          dynamic_cast<rta::logical_task*>(controller.current_task())
              ->name()
              .c_str());

  controller.ndc_pop();
} /* visit() */

void cache_block_drop::visit(controller::depth1::gp_odpo_controller& controller) {
  controller.ndc_push();

  dispatch_d1_cache_interactor(controller.current_task());
  controller.block(nullptr);
  controller.block_manip_collator()->cache_drop_event(true);

  ER_INFO("Dropped block%d in cache%d,task='%s'",
          m_block->id(),
          m_cache->id(),
          dynamic_cast<rta::logical_task*>(controller.current_task())
              ->name()
              .c_str());

  controller.ndc_pop();
} /* visit() */

void cache_block_drop::visit(controller::depth1::gp_omdpo_controller& controller) {
  controller.ndc_push();

  visit(*controller.mdpo_perception()->map());
  dispatch_d1_cache_interactor(controller.current_task());
  controller.block(nullptr);
  controller.block_manip_collator()->cache_drop_event(true);

  ER_INFO("Dropped block%d in cache%d,task='%s'",
          m_block->id(),
          m_cache->id(),
          dynamic_cast<rta::logical_task*>(controller.current_task())
              ->name()
              .c_str());

  controller.ndc_pop();
} /* visit() */

void cache_block_drop::visit(fsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::ekBLOCK_DROP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void cache_block_drop::visit(tasks::depth1::harvester& task) {
  visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void cache_block_drop::visit(controller::depth2::grp_dpo_controller& controller) {
  controller.ndc_push();

  if (dispatch_d2_cache_interactor(controller.current_task(),
                                   controller.cache_sel_matrix())) {
    controller.csel_exception_added(true);
  }
  controller.block_manip_collator()->cache_drop_event(true);
  controller.block(nullptr);
  ER_INFO("Dropped block%d in cache%d,task='%s'",
          m_block->id(),
          m_cache->id(),
          dynamic_cast<rta::logical_task*>(controller.current_task())
              ->name()
              .c_str());

  controller.ndc_pop();
} /* visit() */

void cache_block_drop::visit(controller::depth2::grp_mdpo_controller& controller) {
  controller.ndc_push();

  if (dispatch_d2_cache_interactor(controller.current_task(),
                                   controller.cache_sel_matrix())) {
    controller.csel_exception_added(true);
  }
  visit(*controller.mdpo_perception()->map());
  controller.block_manip_collator()->cache_drop_event(true);
  controller.block(nullptr);
  ER_INFO("Dropped block%d in cache%d,task='%s'",
          m_block->id(),
          m_cache->id(),
          dynamic_cast<rta::logical_task*>(controller.current_task())
              ->name()
              .c_str());

  controller.ndc_pop();
} /* visit() */

void cache_block_drop::visit(controller::depth2::grp_odpo_controller& controller) {
  controller.ndc_push();

  if (dispatch_d2_cache_interactor(controller.current_task(),
                                   controller.cache_sel_matrix())) {
    controller.csel_exception_added(true);
  }
  controller.block_manip_collator()->cache_drop_event(true);
  controller.block(nullptr);
  ER_INFO("Dropped block%d in cache%d,task='%s'",
          m_block->id(),
          m_cache->id(),
          dynamic_cast<rta::logical_task*>(controller.current_task())
              ->name()
              .c_str());

  controller.ndc_pop();
} /* visit() */

void cache_block_drop::visit(
    controller::depth2::grp_omdpo_controller& controller) {
  controller.ndc_push();

  if (dispatch_d2_cache_interactor(controller.current_task(),
                                   controller.cache_sel_matrix())) {
    controller.csel_exception_added(true);
  }
  visit(*controller.mdpo_perception()->map());
  controller.block_manip_collator()->cache_drop_event(true);
  controller.block(nullptr);
  ER_INFO("Dropped block%d in cache%d,task='%s'",
          m_block->id(),
          m_cache->id(),
          dynamic_cast<rta::logical_task*>(controller.current_task())
              ->name()
              .c_str());

  controller.ndc_pop();
} /* visit() */

void cache_block_drop::visit(tasks::depth2::cache_transferer& task) {
  visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

NS_END(detail, events, fordyca);
