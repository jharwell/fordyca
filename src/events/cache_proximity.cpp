/**
 * \file cache_proximity.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/events/cache_proximity.hpp"

#include "cosm/arena/repr/base_cache.hpp"

#include "fordyca/controller/cognitive/d2/birtd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_omdpo_controller.hpp"
#include "fordyca/controller/cognitive/dpo_perception_subsystem.hpp"
#include "fordyca/controller/cognitive/mdpo_perception_subsystem.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/events/cache_found.hpp"
#include "fordyca/events/dynamic_cache_interactor.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/d2/cache_finisher.hpp"
#include "fordyca/tasks/d2/cache_starter.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_proximity::cache_proximity(carepr::base_cache* cache)
    : ER_CLIENT_INIT("fordyca.events.cache_proximity"), m_cache(cache) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_proximity::dispatch_cache_interactor(
    tasks::base_foraging_task* const task) {
  auto* interactor = dynamic_cast<events::dynamic_cache_interactor*>(task);
  ER_ASSERT(
      nullptr != interactor,
      "Non dynamic cache interactor task '%s' received cache proximity event",
      dynamic_cast<cta::logical_task*>(task)->name().c_str());
  interactor->accept(*this);
} /* dispatch_cache_interactor() */

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void cache_proximity::visit(controller::cognitive::d2::birtd_dpo_controller& c) {
  c.ndc_pusht();

  ER_INFO("Abort block drop: cache%d proximity", m_cache->id().v());

  events::cache_found_visitor found(m_cache);
  found.visit(*c.dpo_perception()->dpo_store());

  dispatch_cache_interactor(c.current_task());

  c.ndc_pop();
} /* visit() */

void cache_proximity::visit(controller::cognitive::d2::birtd_mdpo_controller& c) {
  c.ndc_pusht();

  ER_INFO("Abort block drop: cache%d proximity", m_cache->id().v());

  events::cache_found_visitor found(m_cache);
  found.visit(*c.mdpo_perception()->map());

  dispatch_cache_interactor(c.current_task());

  c.ndc_pop();
} /* visit() */

void cache_proximity::visit(controller::cognitive::d2::birtd_odpo_controller& c) {
  c.ndc_pusht();

  ER_INFO("Abort block drop: cache%d proximity", m_cache->id().v());

  events::cache_found_visitor found(m_cache);
  found.visit(*c.dpo_perception()->dpo_store());

  dispatch_cache_interactor(c.current_task());

  c.ndc_pop();
} /* visit() */

void cache_proximity::visit(controller::cognitive::d2::birtd_omdpo_controller& c) {
  c.ndc_pusht();

  ER_INFO("Abort block drop: cache%d proximity", m_cache->id().v());

  events::cache_found_visitor found(m_cache);
  found.visit(*c.mdpo_perception()->map());

  dispatch_cache_interactor(c.current_task());

  c.ndc_pop();
} /* visit() */

void cache_proximity::visit(tasks::d2::cache_finisher& task) {
  visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

void cache_proximity::visit(tasks::d2::cache_starter& task) {
  visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */

void cache_proximity::visit(fsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekCACHE_PROXIMITY,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

NS_END(detail, events, fordyca);
