/**
 * \file nest_block_drop.cpp
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
#include "fordyca/events/nest_block_drop.hpp"

#include "cosm/repr/base_block2D.hpp"

#include "fordyca/controller/depth0/crw_controller.hpp"
#include "fordyca/controller/depth0/dpo_controller.hpp"
#include "fordyca/controller/depth0/mdpo_controller.hpp"
#include "fordyca/controller/depth0/odpo_controller.hpp"
#include "fordyca/controller/depth0/omdpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_dpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_mdpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_odpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_omdpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_dpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_odpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_omdpo_controller.hpp"
#include "fordyca/ds/arena_map.hpp"
#include "fordyca/ds/cell2D.hpp"
#include "fordyca/fsm/depth0/crw_fsm.hpp"
#include "fordyca/fsm/depth0/dpo_fsm.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/depth0/generalist.hpp"
#include "fordyca/tasks/depth1/collector.hpp"
#include "fordyca/tasks/depth1/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
nest_block_drop::nest_block_drop(std::unique_ptr<crepr::base_block2D> robot_block,
                                 const rtypes::timestep& t)
    : ER_CLIENT_INIT("fordyca.events.nest_block_drop"),
      mc_timestep(t),
      m_robot_block(std::move(robot_block)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nest_block_drop::dispatch_nest_interactor(
    tasks::base_foraging_task* const task) {
  RCSW_UNUSED auto* polled = dynamic_cast<cta::polled_task*>(task);
  auto interactor = dynamic_cast<events::nest_interactor*>(task);
  ER_ASSERT(nullptr != interactor,
            "Non nest-interactor task %s causing nest block drop",
            polled->name().c_str());
  interactor->accept(*this);
} /* dispatch_nest_interactor() */

/*******************************************************************************
 * Depth0 Foraging
 ******************************************************************************/
void nest_block_drop::visit(ds::arena_map& map) {
  ER_ASSERT(rtypes::constants::kNoUUID != m_robot_block->robot_id(),
            "Undefined robot index");

  std::scoped_lock lock2(map.block_mtx());
  std::scoped_lock lock1(map.grid_mtx());

  /*
   * The robot owns a unique copy of a block originally from the arena, so we
   * need to look it up rather than implicitly converting its unique_ptr to a
   * shared_ptr and distributing it--this will cause lots of problems later.
   */
  auto it =
      std::find_if(map.blocks().begin(), map.blocks().end(), [&](const auto& b) {
        return m_robot_block->id() == b->id();
      });
  ER_ASSERT(map.blocks().end() != it,
            "Robot block%d not found in arena map blocks",
            m_robot_block->id().v());
  m_arena_block = *it;
  map.distribute_single_block(m_arena_block);
  visit(*m_arena_block);
} /* visit() */

void nest_block_drop::visit(crepr::base_block2D& block) {
  block.reset_metrics();
  block.distribution_time(mc_timestep);
} /* visit() */

void nest_block_drop::visit(controller::depth0::crw_controller& controller) {
  controller.ndc_pusht();
  visit(*controller.fsm());
  controller.block(nullptr);
  controller.block_manip_collator()->free_drop_event(true);

  ER_INFO("Dropped block%d in nest", m_arena_block->id().v());
  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(fsm::depth0::crw_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_DROP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void nest_block_drop::visit(controller::depth0::dpo_controller& controller) {
  controller.ndc_pusht();

  visit(*controller.fsm());
  controller.block(nullptr);
  controller.block_manip_collator()->free_drop_event(true);
  ER_INFO("Dropped block%d in nest", m_arena_block->id().v());

  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(controller::depth0::odpo_controller& controller) {
  controller.ndc_pusht();

  visit(*controller.fsm());
  controller.block(nullptr);
  controller.block_manip_collator()->free_drop_event(true);
  ER_INFO("Dropped block%d in nest", m_arena_block->id().v());

  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(fsm::depth0::dpo_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_DROP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void nest_block_drop::visit(controller::depth0::mdpo_controller& controller) {
  controller.ndc_pusht();

  visit(*controller.fsm());
  controller.block(nullptr);
  controller.block_manip_collator()->free_drop_event(true);
  ER_INFO("Dropped block%d in nest", m_arena_block->id().v());

  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(controller::depth0::omdpo_controller& controller) {
  controller.ndc_pusht();

  visit(*controller.fsm());
  controller.block(nullptr);
  controller.block_manip_collator()->free_drop_event(true);
  ER_INFO("Dropped block%d in nest", m_arena_block->id().v());

  controller.ndc_pop();
} /* visit() */

/*******************************************************************************
 * Depth1 Foraging
 ******************************************************************************/
void nest_block_drop::visit(controller::depth1::bitd_dpo_controller& controller) {
  controller.ndc_pusht();

  controller.block(nullptr);
  dispatch_nest_interactor(controller.current_task());
  controller.block_manip_collator()->free_drop_event(true);
  ER_INFO("Dropped block%d in nest", m_arena_block->id().v());

  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(controller::depth1::bitd_odpo_controller& controller) {
  controller.ndc_pusht();

  controller.block(nullptr);
  dispatch_nest_interactor(controller.current_task());
  controller.block_manip_collator()->free_drop_event(true);
  ER_INFO("Dropped block%d in nest", m_arena_block->id().v());

  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(controller::depth1::bitd_mdpo_controller& controller) {
  controller.ndc_pusht();

  controller.block(nullptr);
  dispatch_nest_interactor(controller.current_task());
  controller.block_manip_collator()->free_drop_event(true);
  ER_INFO("Dropped block%d in nest", m_arena_block->id().v());

  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(
    controller::depth1::bitd_omdpo_controller& controller) {
  controller.ndc_pusht();

  controller.block(nullptr);
  dispatch_nest_interactor(controller.current_task());
  controller.block_manip_collator()->free_drop_event(true);
  ER_INFO("Dropped block%d in nest", m_arena_block->id().v());

  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(tasks::depth0::generalist& task) {
  visit(*static_cast<fsm::depth0::free_block_to_nest_fsm*>(task.mechanism()));
} /* visit() */

void nest_block_drop::visit(tasks::depth1::collector& task) {
  visit(*static_cast<fsm::depth1::cached_block_to_nest_fsm*>(task.mechanism()));
} /* visit() */

void nest_block_drop::visit(fsm::depth1::cached_block_to_nest_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_DROP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void nest_block_drop::visit(fsm::depth0::free_block_to_nest_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_DROP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void nest_block_drop::visit(
    controller::depth2::birtd_mdpo_controller& controller) {
  controller.ndc_pusht();

  controller.block(nullptr);
  dispatch_nest_interactor(controller.current_task());
  controller.block_manip_collator()->free_drop_event(true);
  ER_INFO("Dropped block%d in nest", m_arena_block->id().v());

  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(controller::depth2::birtd_dpo_controller& controller) {
  controller.ndc_pusht();

  controller.block(nullptr);
  dispatch_nest_interactor(controller.current_task());
  controller.block_manip_collator()->free_drop_event(true);
  ER_INFO("Dropped block%d in nest", m_arena_block->id().v());

  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(
    controller::depth2::birtd_odpo_controller& controller) {
  controller.ndc_pusht();

  controller.block(nullptr);
  dispatch_nest_interactor(controller.current_task());
  controller.block_manip_collator()->free_drop_event(true);
  ER_INFO("Dropped block%d in nest", m_arena_block->id().v());

  controller.ndc_pop();
} /* visit() */

void nest_block_drop::visit(
    controller::depth2::birtd_omdpo_controller& controller) {
  controller.ndc_pusht();

  controller.block(nullptr);
  dispatch_nest_interactor(controller.current_task());
  controller.block_manip_collator()->free_drop_event(true);
  ER_INFO("Dropped block%d in nest", m_arena_block->id().v());

  controller.ndc_pop();
} /* visit() */

NS_END(detail, events, fordyca);
