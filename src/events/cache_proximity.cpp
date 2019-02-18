/**
 * @file cache_proximity.cpp
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
#include "fordyca/events/cache_proximity.hpp"
#include "fordyca/controller/depth2/grp_dpo_controller.hpp"
#include "fordyca/controller/depth2/grp_mdpo_controller.hpp"
#include "fordyca/controller/dpo_perception_subsystem.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/events/cache_found.hpp"
#include "fordyca/events/dynamic_cache_interactor.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/repr/base_cache.hpp"
#include "fordyca/tasks/depth2/cache_finisher.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_proximity::cache_proximity(std::shared_ptr<repr::base_cache> cache)
    : ER_CLIENT_INIT("fordyca.events.cache_proximity"), m_cache(cache) {}

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void cache_proximity::visit(controller::depth2::grp_dpo_controller& c) {
  c.ndc_push();

  ER_INFO("Abort block drop: cache%d proximity", m_cache->id());

  events::cache_found found(m_cache);
  c.dpo_perception()->dpo_store()->accept(found);

  auto* task = dynamic_cast<tasks::depth2::cache_finisher*>(c.current_task());
  ER_ASSERT(nullptr != task,
            "Non cache finisher task '%s' received cache proximity event",
            dynamic_cast<ta::logical_task*>(task)->name().c_str());

  task->accept(*this);

  c.ndc_pop();
} /* visit() */

void cache_proximity::visit(controller::depth2::grp_mdpo_controller& c) {
  c.ndc_push();

  ER_INFO("Abort block drop: cache%d proximity", m_cache->id());

  events::cache_found found(m_cache);
  c.mdpo_perception()->map()->accept(found);

  auto* task = dynamic_cast<tasks::depth2::cache_finisher*>(c.current_task());
  ER_ASSERT(nullptr != task,
            "Non cache finisher task '%s' received cache proximity event",
            dynamic_cast<ta::logical_task*>(task)->name().c_str());

  task->accept(*this);

  c.ndc_pop();
} /* visit() */

void cache_proximity::visit(tasks::depth2::cache_finisher& task) {
  static_cast<fsm::block_to_goal_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

void cache_proximity::visit(fsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::CACHE_PROXIMITY,
                   rfsm::event_type::NORMAL);
} /* visit() */

NS_END(events, fordyca);
