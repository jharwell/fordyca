/**
 * @file cache_appeared.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/events/cache_appeared.hpp"
#include "fordyca/controller/depth2/greedy_recpart_controller.hpp"
#include "fordyca/fsm/depth1/block_to_goal_fsm.hpp"
#include "fordyca/tasks/depth2/cache_finisher.hpp"
#include "fordyca/tasks/depth2/cache_starter.hpp"
#include "fordyca/tasks/depth2/dynamic_cache_interactor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_appeared::cache_appeared(uint cache_id)
    : ER_CLIENT_INIT("fordyca.events.cache_appeared"), m_cache_id(cache_id) {}

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void cache_appeared::visit(
    controller::depth2::greedy_recpart_controller& controller) {
  controller.ndc_push();
  ER_INFO("Abort block drop: cache%d appeared", m_cache_id);

  auto* task = dynamic_cast<tasks::depth2::dynamic_cache_interactor*>(
      controller.current_task());
  ER_ASSERT(
      nullptr != task,
      "Non existing cache interactor task %s received cache appeared event",
      dynamic_cast<ta::logical_task*>(task)->name().c_str());

  task->accept(*this);
  controller.ndc_pop();
} /* visit() */

void cache_appeared::visit(tasks::depth2::cache_starter& task) {
  static_cast<fsm::depth1::block_to_goal_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

void cache_appeared::visit(tasks::depth2::cache_finisher& task) {
  static_cast<fsm::depth1::block_to_goal_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

void cache_appeared::visit(fsm::depth1::block_to_goal_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::CACHE_APPEARED,
                   state_machine::event_type::NORMAL);
} /* visit() */

NS_END(events, fordyca);