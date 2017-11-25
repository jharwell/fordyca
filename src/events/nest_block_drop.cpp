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
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "fordyca/representation/arena_map.hpp"
#include "fordyca/controller/depth0/stateless_foraging_controller.hpp"
#include "fordyca/controller/depth0/stateful_foraging_controller.hpp"
#include "fordyca/controller/depth1/foraging_controller.hpp"
#include "fordyca/metrics/collectors/block_metrics_collector.hpp"
#include "fordyca/fsm/depth0/stateless_foraging_fsm.hpp"
#include "fordyca/fsm/depth0/stateful_foraging_fsm.hpp"
#include "fordyca/tasks/foraging_task.hpp"
#include "fordyca/tasks/generalist.hpp"
#include "fordyca/tasks/collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
nest_block_drop::nest_block_drop(const std::shared_ptr<rcppsw::er::server>& server,
                                 representation::block* block) :
    client(server), m_block(block) {
  client::insmod("nest_block_drop",
                    rcppsw::er::er_lvl::DIAG,
                    rcppsw::er::er_lvl::NOM);
}

/*******************************************************************************
 * Foraging Support
 ******************************************************************************/
void nest_block_drop::visit(representation::arena_map& map) {
  ER_ASSERT(-1 != m_block->robot_index(), "FATAL: undefined robot index");
  map.distribute_block(m_block, false);
  int index = m_block->robot_index();
  m_block->accept(*this);
  ER_NOM("fb%d dropped block%d in nest", index, m_block->id());
} /* visit() */

void nest_block_drop::visit(metrics::collectors::block_metrics_collector& collector) {
  collector.collect(*m_block);
} /* visit() */

/*******************************************************************************
 * Stateless Foraging
 ******************************************************************************/
void nest_block_drop::visit(representation::block& block) {
  block.reset();
} /* visit() */

void nest_block_drop::visit(controller::depth0::stateless_foraging_controller& controller) {
  controller.fsm()->accept(*this);
  controller.block(nullptr);
  ER_NOM("stateless_foraging_controller: %s dropped block%d in nest",
         controller.GetId().c_str(), m_block->id());
} /* visit() */


void nest_block_drop::visit(fsm::depth0::stateless_foraging_fsm& fsm) {
  ER_NOM("stateless_foraging_fsm: register nest_block_drop event");
  fsm.inject_event(controller::foraging_signal::BLOCK_DROP,
                   state_machine::event_type::NORMAL);
} /* visit() */

/*******************************************************************************
 * Stateful Foraging
 ******************************************************************************/
void nest_block_drop::visit(controller::depth0::stateful_foraging_controller& controller) {
  controller.current_task()->accept(*this);
  controller.block(nullptr);
  ER_NOM("stateful_foraging_controller: %s dropped block%d in nest",
         controller.GetId().c_str(), m_block->id());
} /* visit() */

void nest_block_drop::visit(fsm::depth0::stateful_foraging_fsm& fsm) {
  ER_NOM("stateful_foraging_fsm: register nest_block_drop event");
  fsm.inject_event(controller::foraging_signal::BLOCK_DROP,
                   state_machine::event_type::NORMAL);
} /* visit() */

/*******************************************************************************
 * Depth1 Foraging
 ******************************************************************************/
void nest_block_drop::visit(controller::depth1::foraging_controller& controller) {
  controller.block(nullptr);
  controller.current_task()->accept(*this);

  ER_NOM("depth1_foraging_controller: %s dropped block%d in nest",
         controller.GetId().c_str(), m_block->id());
} /* visit() */

void nest_block_drop::visit(tasks::generalist& task) {
  static_cast<fsm::depth0::stateful_foraging_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

void nest_block_drop::visit(tasks::collector& task) {
  static_cast<fsm::block_to_nest_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

void nest_block_drop::visit(fsm::block_to_nest_fsm& fsm) {
  ER_NOM("block_to_nest_fsm: register nest_block_drop event");
  fsm.inject_event(controller::foraging_signal::BLOCK_DROP,
                   state_machine::event_type::NORMAL);
} /* visit() */


NS_END(events, fordyca);
