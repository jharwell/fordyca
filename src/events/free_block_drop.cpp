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
#include "fordyca/controller/depth1/greedy_partitioning_controller.hpp"
#include "fordyca/controller/depth2/greedy_recpart_controller.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/ds/arena_map.hpp"
#include "fordyca/ds/cell2D.hpp"
#include "fordyca/ds/perceived_arena_map.hpp"
#include "fordyca/events/cache_block_drop.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/tasks/depth1/foraging_task.hpp"
#include "fordyca/tasks/depth2/cache_finisher.hpp"
#include "fordyca/tasks/depth2/cache_starter.hpp"
#include "fordyca/tasks/depth2/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);
using ds::arena_grid;
using ds::occupancy_grid;
namespace rfsm = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
free_block_drop::free_block_drop(
    const std::shared_ptr<representation::base_block>& block,
    const rmath::vector2u& coord,
    double resolution)
    : cell_op(coord),
      ER_CLIENT_INIT("fordyca.events.free_block_drop"),
      m_resolution(resolution),
      m_block(block) {}

/*******************************************************************************
 * Depth0
 ******************************************************************************/
void free_block_drop::visit(ds::cell2D& cell) {
  cell.entity(m_block);
  m_block->accept(*this);
  cell.fsm().accept(*this);
} /* visit() */

void free_block_drop::visit(fsm::cell2D_fsm& fsm) {
  fsm.event_block_drop();
} /* visit() */

void free_block_drop::visit(representation::base_block& block) {
  block.reset_robot_id();

  block.real_loc(rmath::uvec2dvec(cell_op::coord(), m_resolution));
  block.discrete_loc(cell_op::coord());
} /* visit() */

void free_block_drop::visit(ds::arena_map& map) {
  ds::cell2D& cell = map.access<arena_grid::kCell>(cell_op::coord());

  if (cell.state_has_cache()) {
    cache_block_drop op(m_block,
                        std::static_pointer_cast<representation::arena_cache>(
                            cell.cache()),
                        m_resolution);
    map.accept(op);
    return;
  }

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
   * This was a terrible bug to track down.
   */
  if (cell.state_has_block()) {
    map.distribute_single_block(m_block);
  }
  /*
   * Cell does not have a block/cache on it; either empty or unknown (base
   * case).
   */
  cell.accept(*this);
} /* visit() */

/*******************************************************************************
 * Depth1
 ******************************************************************************/
void free_block_drop::visit(
    controller::depth1::greedy_partitioning_controller& controller) {
  controller.block(nullptr);
} /* visit() */

/*******************************************************************************
 * Depth2
 ******************************************************************************/
void free_block_drop::visit(
    controller::depth2::greedy_recpart_controller& controller) {
  controller.ndc_push();

  auto* polled = dynamic_cast<ta::polled_task*>(controller.current_task());
  auto* task =
      dynamic_cast<events::free_block_interactor*>(controller.current_task());
  if (nullptr != task) {
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
              m_block->real_loc().to_str().c_str(),
              polled->name().c_str());
      controller.block_sel_matrix()->sel_exception_add(m_block->id());
      controller.bsel_exception_added(true);
    }
    task->accept(*this);
  }
  ER_INFO("Dropped block%d@%s,task='%s'",
          m_block->id(),
          m_block->real_loc().to_str().c_str(),
          polled->name().c_str());
  controller.block(nullptr);
  controller.ndc_pop();
} /* visit() */

void free_block_drop::visit(ds::perceived_arena_map& map) {
  ds::cell2D& cell = map.access<occupancy_grid::kCell>(cell_op::coord());
  cell.accept(*this);
} /* visit() */

void free_block_drop::visit(tasks::depth2::cache_starter& task) {
  static_cast<fsm::block_to_goal_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

void free_block_drop::visit(tasks::depth2::cache_finisher& task) {
  static_cast<fsm::block_to_goal_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

void free_block_drop::visit(fsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_DROP,
                   rfsm::event_type::NORMAL);
} /* visit() */

NS_END(events, fordyca);
