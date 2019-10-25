/**
 * @file free_block_drop.cpp
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
#include "fordyca/events/free_block_drop.hpp"

#include "fordyca/controller/base_perception_subsystem.hpp"
#include "fordyca/controller/block_sel_matrix.hpp"
#include "fordyca/controller/depth0/dpo_controller.hpp"
#include "fordyca/controller/depth0/mdpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_dpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_mdpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_odpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_omdpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_dpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_odpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_omdpo_controller.hpp"
#include "fordyca/ds/arena_map.hpp"
#include "fordyca/ds/cell2D.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/events/cache_block_drop.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/repr/base_block.hpp"
#include "fordyca/tasks/depth1/foraging_task.hpp"
#include "fordyca/tasks/depth2/cache_finisher.hpp"
#include "fordyca/tasks/depth2/cache_starter.hpp"
#include "fordyca/tasks/depth2/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);
using ds::arena_grid;
using ds::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
free_block_drop::free_block_drop(const std::shared_ptr<repr::base_block>& block,
                                 const rmath::vector2u& coord,
                                 rtypes::discretize_ratio resolution,
                                 bool cache_lock)
    : ER_CLIENT_INIT("fordyca.events.free_block_drop"),
      cell_op(coord),
      mc_resolution(resolution),
      mc_cache_lock(cache_lock),
      m_block(block) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool free_block_drop::dispatch_free_block_interactor(
    tasks::base_foraging_task* const task,
    controller::block_sel_matrix* const bsel_matrix) {
  auto* polled = dynamic_cast<rta::polled_task*>(task);
  auto* interactor = dynamic_cast<events::free_block_interactor*>(task);
  bool ret = false;

  if (nullptr != interactor) {
    /*
     * If this is true then we know the current task must be a cache starter or
     * a cache finisher, so we need to make a note of the block we are dropping
     * so we don't just turn around and pick it up when we start our next task.
     *
     * If we are performing a free block drop because we have just aborted our
     * task, then obviously no need to do that.
     */
    if (tasks::depth2::foraging_task::task_in_depth2(polled) &&
        !polled->task_aborted()) {
      ER_INFO("Added block%d@%s to exception list,task='%s'",
              m_block->id(),
              m_block->rloc().to_str().c_str(),
              polled->name().c_str());
      bsel_matrix->sel_exception_add(m_block->id());
      ret = true;
    }
    interactor->accept(*this);
  }
  return ret;
} /* dispatch_free_block_interactor() */

/*******************************************************************************
 * Depth0
 ******************************************************************************/
void free_block_drop::visit(ds::cell2D& cell) {
  visit(*m_block);
  visit(cell.fsm());
  cell.entity(m_block);
} /* visit() */

void free_block_drop::visit(fsm::cell2D_fsm& fsm) {
  fsm.event_block_drop();
} /* visit() */

void free_block_drop::visit(repr::base_block& block) {
  block.reset_robot_id();

  block.rloc(rmath::uvec2dvec(cell_op::coord(), mc_resolution.v()));
  block.dloc(cell_op::coord());
} /* visit() */

void free_block_drop::visit(ds::arena_map& map) {
  ds::cell2D& cell = map.access<arena_grid::kCell>(cell_op::coord());

  /*
   * Dropping a block onto a cell that already contains a single block (but not
   * a cache) does not work, so we have to fudge it and just distribute the
   * block. Failing to do this results robots that are carrying a block and that
   * abort their current task causing the cell that they drop the block onto to
   * go into a HAS_CACHE state, when the cell entity is not a cache. This
   * unsurprisingly causes a segfault later.
   *
   * Even in depth2, when dynamic cache creation is enabled, robots drop blocks
   * NEXT to others to start caches, NOT on top of them.
   *
   * I already changed this once and had to track down and re-fix it. DO NOT
   * CHANGE AGAIN UNLESS REALLY REALLY SURE WHAT I AM DOING.
   */
  if (cell.state_has_cache()) {
    if (mc_cache_lock) {
      map.cache_mtx().lock();
    }
    cache_block_drop op(m_block,
                        std::static_pointer_cast<repr::arena_cache>(cell.cache()),
                        mc_resolution);
    op.visit(map);
    if (mc_cache_lock) {
      map.cache_mtx().unlock();
    }
  } else if (cell.state_has_block()) {
    map.distribute_single_block(m_block);
  } else {
    /*
     * Cell does not have a block/cache on it, so it is safe to drop the block
     * on it and change the cell state.
     */
    visit(cell);
  }
} /* visit() */

/*******************************************************************************
 * Depth1
 ******************************************************************************/
void free_block_drop::visit(controller::depth1::bitd_mdpo_controller& controller) {
  controller.block(nullptr);
} /* visit() */

void free_block_drop::visit(
    controller::depth1::bitd_omdpo_controller& controller) {
  controller.block(nullptr);
} /* visit() */

void free_block_drop::visit(controller::depth1::bitd_dpo_controller& controller) {
  controller.block(nullptr);
} /* visit() */

void free_block_drop::visit(controller::depth1::bitd_odpo_controller& controller) {
  controller.block(nullptr);
} /* visit() */

/*******************************************************************************
 * Depth2
 ******************************************************************************/
void free_block_drop::visit(
    controller::depth2::birtd_mdpo_controller& controller) {
  controller.ndc_push();

  if (dispatch_free_block_interactor(controller.current_task(),
                                     controller.block_sel_matrix())) {
    controller.bsel_exception_added(true);
  }
  controller.block(nullptr);

  controller.ndc_pop();
} /* visit() */

void free_block_drop::visit(controller::depth2::birtd_dpo_controller& controller) {
  controller.ndc_push();

  if (dispatch_free_block_interactor(controller.current_task(),
                                     controller.block_sel_matrix())) {
    controller.bsel_exception_added(true);
  }
  controller.block(nullptr);

  controller.ndc_pop();
} /* visit() */

void free_block_drop::visit(
    controller::depth2::birtd_omdpo_controller& controller) {
  controller.ndc_push();

  if (dispatch_free_block_interactor(controller.current_task(),
                                     controller.block_sel_matrix())) {
    controller.bsel_exception_added(true);
  }
  controller.block(nullptr);

  controller.ndc_pop();
} /* visit() */

void free_block_drop::visit(
    controller::depth2::birtd_odpo_controller& controller) {
  controller.ndc_push();

  if (dispatch_free_block_interactor(controller.current_task(),
                                     controller.block_sel_matrix())) {
    controller.bsel_exception_added(true);
  }
  controller.block(nullptr);

  controller.ndc_pop();
} /* visit() */

void free_block_drop::visit(ds::dpo_semantic_map& map) {
  ds::cell2D& cell = map.access<occupancy_grid::kCell>(cell_op::coord());
  visit(cell);
} /* visit() */

void free_block_drop::visit(tasks::depth2::cache_starter& task) {
  visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

void free_block_drop::visit(tasks::depth2::cache_finisher& task) {
  visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

void free_block_drop::visit(fsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_DROP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

NS_END(detail, events, fordyca);
