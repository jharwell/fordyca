/**
 * \file block_proximity.cpp
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
#include "fordyca/events/block_proximity.hpp"

#include "cosm/repr/base_block2D.hpp"

#include "fordyca/controller/depth2/birtd_dpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_odpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_omdpo_controller.hpp"
#include "fordyca/controller/dpo_perception_subsystem.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/events/block_found.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/depth2/cache_starter.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_proximity::block_proximity(const std::shared_ptr<crepr::base_block2D>& block)
    : ER_CLIENT_INIT("fordyca.events.block_proximity"), m_block(block) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_proximity::dispatch_cache_starter(
    tasks::base_foraging_task* const task) {
  auto* starter = dynamic_cast<tasks::depth2::cache_starter*>(task);
  ER_ASSERT(nullptr != starter,
            "Non cache starter task %s received block proximity event",
            dynamic_cast<cta::logical_task*>(task)->name().c_str());
  starter->accept(*this);
} /* dispatch_cache_starter() */

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void block_proximity::visit(controller::depth2::birtd_dpo_controller& c) {
  c.ndc_push();

  ER_INFO("Abort block drop: block%d proximity", m_block->id().v());
  events::block_found_visitor found_op(m_block);
  found_op.visit(*c.dpo_perception()->dpo_store());
  dispatch_cache_starter(c.current_task());

  c.ndc_pop();
} /* visit() */

void block_proximity::visit(controller::depth2::birtd_mdpo_controller& c) {
  c.ndc_push();

  ER_INFO("Abort block drop: block%d proximity", m_block->id().v());
  events::block_found_visitor found_op(m_block);
  found_op.visit(*c.mdpo_perception()->map());
  dispatch_cache_starter(c.current_task());

  c.ndc_pop();
} /* visit() */

void block_proximity::visit(controller::depth2::birtd_odpo_controller& c) {
  c.ndc_push();

  ER_INFO("Abort block drop: block%d proximity", m_block->id().v());
  events::block_found_visitor found_op(m_block);
  found_op.visit(*c.dpo_perception()->dpo_store());
  dispatch_cache_starter(c.current_task());

  c.ndc_pop();
} /* visit() */

void block_proximity::visit(controller::depth2::birtd_omdpo_controller& c) {
  c.ndc_push();

  ER_INFO("Abort block drop: block%d proximity", m_block->id().v());
  events::block_found_visitor found_op(m_block);
  found_op.visit(*c.mdpo_perception()->map());
  dispatch_cache_starter(c.current_task());

  c.ndc_pop();
} /* visit() */

void block_proximity::visit(tasks::depth2::cache_starter& task) {
  visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

void block_proximity::visit(fsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_PROXIMITY,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

NS_END(detail, events, fordyca);
