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
#include "fordyca/controller/cognitive/d2/events/block_proximity.hpp"

#include "cosm/repr/base_block3D.hpp"

#include "fordyca/controller/cognitive/d2/birtd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_omdpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/events/block_found.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/d2/cache_starter.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_proximity::block_proximity(crepr::base_block3D* block)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d2.events.block_proximity"),
      m_block(block) {}

/*******************************************************************************
 * Controllers
 ******************************************************************************/
void block_proximity::visit(fccd2::birtd_dpo_controller& c) {
  c.ndc_pusht();

  ER_INFO("Abort block drop: block%d proximity", m_block->id().v());
  fccd2::events::block_found_visitor found_op(m_block);
  found_op.visit(c);
  dispatch_cache_starter(c.current_task());

  c.ndc_pop();
} /* visit() */

void block_proximity::visit(fccd2::birtd_mdpo_controller& c) {
  c.ndc_pusht();

  ER_INFO("Abort block drop: block%d proximity", m_block->id().v());
  fccd2::events::block_found_visitor found_op(m_block);
  found_op.visit(c);
  dispatch_cache_starter(c.current_task());

  c.ndc_pop();
} /* visit() */

void block_proximity::visit(fccd2::birtd_odpo_controller& c) {
  c.ndc_pusht();

  ER_INFO("Abort block drop: block%d proximity", m_block->id().v());
  fccd2::events::block_found_visitor found_op(m_block);
  found_op.visit(c);
  dispatch_cache_starter(c.current_task());

  c.ndc_pop();
} /* visit() */

void block_proximity::visit(fccd2::birtd_omdpo_controller& c) {
  c.ndc_pusht();

  ER_INFO("Abort block drop: block%d proximity", m_block->id().v());
  fccd2::events::block_found_visitor found_op(m_block);
  found_op.visit(c);
  dispatch_cache_starter(c.current_task());

  c.ndc_pop();
} /* visit() */

/*******************************************************************************
 * Tasks
 ******************************************************************************/
void block_proximity::visit(ftasks::d2::cache_starter& task) {
  visit(*static_cast<ffsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

/*******************************************************************************
 * FSMs
 ******************************************************************************/
void block_proximity::visit(ffsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(ffsm::foraging_signal::ekBLOCK_PROXIMITY,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_proximity::dispatch_cache_starter(
    tasks::base_foraging_task* const task) {
  auto* starter = dynamic_cast<tasks::d2::cache_starter*>(task);
  ER_ASSERT(nullptr != starter,
            "Non cache starter task %s received block proximity event",
            dynamic_cast<cta::logical_task*>(task)->name().c_str());
  starter->accept(*this);
} /* dispatch_cache_starter() */

NS_END(events, d2, cognitive, controller, fordyca);
