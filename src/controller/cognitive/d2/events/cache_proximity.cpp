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
#include "fordyca/controller/cognitive/d2/events/cache_proximity.hpp"

#include "cosm/arena/repr/base_cache.hpp"

#include "fordyca/controller/cognitive/d2/birtd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/birtd_omdpo_controller.hpp"
#include "fordyca/controller/cognitive/d2/events/cache_found.hpp"
#include "fordyca/events/dynamic_cache_interactor.hpp"
#include "fordyca/events/existing_cache_interactor.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/subsystem/perception/dpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/ds/dpo_semantic_map.hpp"
#include "fordyca/subsystem/perception/mdpo_perception_subsystem.hpp"
#include "fordyca/tasks/d2/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_proximity::cache_proximity(carepr::base_cache* cache)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d2.events.cache_proximity"),
      m_cache(cache) {}

/*******************************************************************************
 * Controllers
 ******************************************************************************/
void cache_proximity::visit(fccd2::birtd_dpo_controller& c) {
  c.ndc_uuid_push();

  ER_INFO("Abort block drop: cache%d proximity", m_cache->id().v());

  fccd2::events::cache_found_visitor found(m_cache);
  found.visit(c);

  dispatch_cache_interactor(c.current_task());

  c.ndc_uuid_pop();
} /* visit() */

void cache_proximity::visit(fccd2::birtd_mdpo_controller& c) {
  c.ndc_uuid_push();

  ER_INFO("Abort block drop: cache%d proximity", m_cache->id().v());

  fccd2::events::cache_found_visitor found(m_cache);
  found.visit(c);

  dispatch_cache_interactor(c.current_task());

  c.ndc_uuid_pop();
} /* visit() */

void cache_proximity::visit(fccd2::birtd_odpo_controller& c) {
  c.ndc_uuid_push();

  ER_INFO("Abort block drop: cache%d proximity", m_cache->id().v());

  fccd2::events::cache_found_visitor found(m_cache);
  found.visit(c);

  dispatch_cache_interactor(c.current_task());

  c.ndc_uuid_pop();
} /* visit() */

void cache_proximity::visit(fccd2::birtd_omdpo_controller& c) {
  c.ndc_uuid_push();

  ER_INFO("Abort block drop: cache%d proximity", m_cache->id().v());

  fccd2::events::cache_found_visitor found(m_cache);
  found.visit(c);

  dispatch_cache_interactor(c.current_task());

  c.ndc_uuid_pop();
} /* visit() */

/*******************************************************************************
 * FSMs
 ******************************************************************************/
void cache_proximity::visit(fsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekCACHE_PROXIMITY,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_proximity::dispatch_cache_interactor(
    tasks::base_foraging_task* const task) {
  auto* interactor = dynamic_cast<fevents::dynamic_cache_interactor*>(task);
  ER_ASSERT(nullptr != interactor,
            "Non dynamic cache interactor task '%s' received cache proximity "
            "event",
            dynamic_cast<cta::logical_task*>(task)->name().c_str());
  interactor->accept(*this);
} /* dispatch_cache_interactor() */

NS_END(events, d2, cognitive, controller, fordyca);
