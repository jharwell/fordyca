/**
 * @file block_proximity.cpp
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
#include "fordyca/events/block_proximity.hpp"
#include "fordyca/controller/base_perception_subsystem.hpp"
#include "fordyca/controller/depth2/greedy_recpart_controller.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/ds/perceived_arena_map.hpp"
#include "fordyca/events/block_found.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/tasks/depth2/cache_starter.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_proximity::block_proximity(
    const std::shared_ptr<representation::base_block>& block)
    : ER_CLIENT_INIT("fordyca.events.block_proximity"), m_block(block) {}

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void block_proximity::visit(controller::depth2::greedy_recpart_controller& c) {
  c.ndc_push();
  ER_INFO("Abort block drop: block%d proximity", m_block->id());
  events::block_found found(m_block);
  c.perception()->map()->accept(found);

  auto* task = dynamic_cast<tasks::depth2::cache_starter*>(c.current_task());
  ER_ASSERT(nullptr != task,
            "Non cache starter task %s received block proximity event",
            dynamic_cast<ta::logical_task*>(task)->name().c_str());
  task->accept(*this);
  c.ndc_pop();
} /* visit() */

void block_proximity::visit(tasks::depth2::cache_starter& task) {
  static_cast<fsm::block_to_goal_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

void block_proximity::visit(fsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_PROXIMITY,
                   state_machine::event_type::NORMAL);
} /* visit() */

NS_END(events, fordyca);
