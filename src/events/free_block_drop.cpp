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
#include <argos3/core/utility/math/vector2.h>

#include "fordyca/controller/depth1/greedy_partitioning_controller.hpp"
#include "fordyca/controller/depth2/greedy_recpart_controller.hpp"
#include "fordyca/ds/arena_map.hpp"
#include "fordyca/ds/cell2D.hpp"
#include "fordyca/events/cache_block_drop.hpp"
#include "fordyca/fsm/depth1/block_to_goal_fsm.hpp"
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

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
free_block_drop::free_block_drop(
    const std::shared_ptr<representation::base_block>& block,
    rcppsw::math::dcoord2 coord,
    double resolution)
    : cell_op(coord.first, coord.second),
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

  rcppsw::math::dcoord2 d(cell_op::x(), cell_op::y());
  block.real_loc(math::dcoord_to_rcoord(d, m_resolution));
  block.discrete_loc(d);
} /* visit() */

void free_block_drop::visit(ds::arena_map& map) {
  ds::cell2D& cell = map.access<arena_grid::kCell>(cell_op::x(), cell_op::y());

  /*
   * @todo We should be able to handle dropping a block on a cell in any
   * state. However, until we get to depth2, dropping a block onto a cell that
   * already contains a single block (but not a cache) does not work, so we have
   * to fudge it and just distribute the block. Failing to do this results
   * robots that are carrying a block and that abort their current task causing
   * the cell that the drop the block onto to go into a HAS_CACHE state, when
   * the cell entity is not a cache.
   *
   * This was a terrible bug to track down.
   */
  if (cell.state_has_cache()) {
    cache_block_drop op(m_block,
                        std::static_pointer_cast<representation::arena_cache>(
                            cell.cache()),
                        m_resolution);
    map.accept(op);
  } else if (cell.state_has_block()) {
    map.distribute_single_block(m_block);
  } else {
    cell.accept(*this);
  }
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

  auto* task = dynamic_cast<tasks::free_block_interactor*>(
      controller.current_task());
ER_ASSERT(nullptr != task,
            "Non free block interactor task %s causing free block drop",
            dynamic_cast<ta::logical_task*>(task)->name().c_str());

  task->accept(*this);
  controller.block(nullptr);
} /* visit() */

void free_block_drop::visit(tasks::depth2::cache_starter& task) {
  static_cast<fsm::depth1::block_to_goal_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

void free_block_drop::visit(tasks::depth2::cache_finisher& task) {
  static_cast<fsm::depth1::block_to_goal_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

void free_block_drop::visit(fsm::depth1::block_to_goal_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_DROP,
                   state_machine::event_type::NORMAL);
} /* visit() */

NS_END(events, fordyca);
