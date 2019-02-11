/**
 * @file nest_block_drop.cpp
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
#include "fordyca/events/nest_block_drop.hpp"
#include "fordyca/controller/depth0/crw_controller.hpp"
#include "fordyca/controller/depth0/mdpo_controller.hpp"
#include "fordyca/controller/depth1/gp_mdpo_controller.hpp"
#include "fordyca/controller/depth2/grp_mdpo_controller.hpp"
#include "fordyca/ds/arena_map.hpp"
#include "fordyca/ds/cell2D.hpp"
#include "fordyca/fsm/depth0/crw_fsm.hpp"
#include "fordyca/fsm/depth0/dpo_fsm.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/tasks/depth0/generalist.hpp"
#include "fordyca/tasks/depth1/collector.hpp"
#include "fordyca/tasks/depth1/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
nest_block_drop::nest_block_drop(std::shared_ptr<representation::base_block> block,
                                 uint timestep)
    : ER_CLIENT_INIT("fordyca.events.nest_block_drop"),
      m_timestep(timestep),
      m_block(block) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nest_block_drop::visit(ds::arena_map& map) {
  ER_ASSERT(-1 != m_block->robot_id(), "Undefined robot index");
  map.distribute_single_block(m_block);
  m_block->accept(*this);
} /* visit() */

/*******************************************************************************
 * Depth0 Foraging
 ******************************************************************************/
void nest_block_drop::visit(representation::base_block& block) {
  block.reset_metrics();
  block.distribution_time(m_timestep);
} /* visit() */

void nest_block_drop::visit(controller::depth0::crw_controller& controller) {
  controller.ndc_push();
  controller.fsm()->accept(*this);
  controller.block(nullptr);
  controller.free_drop_event(true);
  controller.esubsystem()->drop_block();

  ER_INFO("Dropped block%d in nest", m_block->id());
  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(fsm::depth0::crw_fsm& fsm) {
  ER_INFO("DROPPING THE BLOCK................")
  fsm.inject_event(controller::foraging_signal::BLOCK_DROP,
                   state_machine::event_type::NORMAL);
} /* visit() */

void nest_block_drop::visit(controller::depth0::dpo_controller& controller) {
  controller.ndc_push();

  controller.fsm()->accept(*this);
  controller.block(nullptr);
  controller.free_drop_event(true);
  ER_INFO("Dropped block%d in nest", m_block->id());

  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(fsm::depth0::dpo_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_DROP,
                   state_machine::event_type::NORMAL);
} /* visit() */

void nest_block_drop::visit(controller::depth0::mdpo_controller& controller) {
  controller.ndc_push();

  controller.fsm()->accept(*this);
  controller.block(nullptr);
  controller.free_drop_event(true);
  ER_INFO("Dropped block%d in nest", m_block->id());

  controller.ndc_pop();
} /* visit() */

/*******************************************************************************
 * Depth1 Foraging
 ******************************************************************************/
void nest_block_drop::visit(controller::depth1::gp_dpo_controller& controller) {
  controller.ndc_push();

  controller.block(nullptr);
  auto task = dynamic_cast<events::nest_interactor*>(controller.current_task());
  __rcsw_unused auto* polled =
      dynamic_cast<ta::polled_task*>(controller.current_task());
  ER_ASSERT(nullptr != task,
            "Non nest-interactor task %s causing nest block drop",
            polled->name().c_str());
  task->accept(*this);
  controller.free_drop_event(true);
  ER_INFO("Dropped block%d in nest", m_block->id());

  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(controller::depth1::gp_mdpo_controller& controller) {
  controller.ndc_push();

  controller.block(nullptr);
  auto task = dynamic_cast<events::nest_interactor*>(controller.current_task());
  __rcsw_unused auto* polled =
      dynamic_cast<ta::polled_task*>(controller.current_task());
  ER_ASSERT(nullptr != task,
            "Non nest-interactor task %s causing nest block drop",
            polled->name().c_str());
  task->accept(*this);
  controller.free_drop_event(true);
  ER_INFO("Dropped block%d in nest", m_block->id());

  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(tasks::depth0::generalist& task) {
  static_cast<fsm::depth0::free_block_to_nest_fsm*>(task.mechanism())
      ->accept(*this);
} /* visit() */

void nest_block_drop::visit(tasks::depth1::collector& task) {
  static_cast<fsm::depth1::cached_block_to_nest_fsm*>(task.mechanism())
      ->accept(*this);
} /* visit() */

void nest_block_drop::visit(fsm::depth1::cached_block_to_nest_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_DROP,
                   state_machine::event_type::NORMAL);
} /* visit() */

void nest_block_drop::visit(fsm::depth0::free_block_to_nest_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_DROP,
                   state_machine::event_type::NORMAL);
} /* visit() */

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void nest_block_drop::visit(controller::depth2::grp_mdpo_controller& controller) {
  controller.ndc_push();

  controller.block(nullptr);
  __rcsw_unused auto* polled =
      dynamic_cast<ta::polled_task*>(controller.current_task());
  auto task = dynamic_cast<events::nest_interactor*>(controller.current_task());
  ER_ASSERT(nullptr != task,
            "Non nest-interactor task %s causing nest block drop",
            polled->name().c_str());
  task->accept(*this);
  controller.free_drop_event(true);
  ER_INFO("Dropped block%d in nest", m_block->id());

  controller.ndc_pop();
} /* visit() */

NS_END(events, fordyca);
