/**
 * \file block_proximity.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/cognitive/d2/events/block_proximity.hpp"

#include "cosm/repr/sim_block3D.hpp"

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
block_proximity::block_proximity(crepr::sim_block3D* block)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d2.events.block_proximity"),
      m_block(block) {}

/*******************************************************************************
 * Controllers
 ******************************************************************************/
void block_proximity::visit(fccd2::birtd_dpo_controller& c) {
  c.ndc_uuid_push();

  ER_INFO("Abort block drop: block%d proximity", m_block->id().v());
  fccd2::events::block_found_visitor found_op(m_block);
  found_op.visit(c);
  dispatch_cache_starter(c.current_task());

  c.ndc_uuid_pop();
} /* visit() */

void block_proximity::visit(fccd2::birtd_mdpo_controller& c) {
  c.ndc_uuid_push();

  ER_INFO("Abort block drop: block%d proximity", m_block->id().v());
  fccd2::events::block_found_visitor found_op(m_block);
  found_op.visit(c);
  dispatch_cache_starter(c.current_task());

  c.ndc_uuid_pop();
} /* visit() */

void block_proximity::visit(fccd2::birtd_odpo_controller& c) {
  c.ndc_uuid_push();

  ER_INFO("Abort block drop: block%d proximity", m_block->id().v());
  fccd2::events::block_found_visitor found_op(m_block);
  found_op.visit(c);
  dispatch_cache_starter(c.current_task());

  c.ndc_uuid_pop();
} /* visit() */

void block_proximity::visit(fccd2::birtd_omdpo_controller& c) {
  c.ndc_uuid_push();

  ER_INFO("Abort block drop: block%d proximity", m_block->id().v());
  fccd2::events::block_found_visitor found_op(m_block);
  found_op.visit(c);
  dispatch_cache_starter(c.current_task());

  c.ndc_uuid_pop();
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
