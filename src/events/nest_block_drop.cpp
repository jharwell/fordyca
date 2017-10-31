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
#include "fordyca/controller/random_foraging_controller.hpp"
#include "fordyca/controller/memory_foraging_controller.hpp"
#include "fordyca/support/block_stat_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
nest_block_drop::nest_block_drop(const std::shared_ptr<rcppsw::common::er_server>& server,
                                 representation::block* block) :
    er_client(server), m_block(block) {
  er_client::insmod("nest_block_drop",
                    rcppsw::common::er_lvl::DIAG,
                    rcppsw::common::er_lvl::NOM);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nest_block_drop::visit(representation::arena_map& map) {
  map.distribute_block(m_block, false);
  ER_ASSERT(-1 != m_block->robot_index(), "FATAL: undefined robot index");
  int index = m_block->robot_index();
  m_block->accept(*this);
  ER_NOM("fb%d dropped block%d in nest", index, m_block->id());
} /* visit() */

void nest_block_drop::visit(support::block_stat_collector& collector) {
  collector.inc_total_collected();
  collector.inc_total_carries(m_block->carries());
} /* visit() */

void nest_block_drop::visit(representation::block& block) {
  block.reset();
} /* visit() */

void nest_block_drop::visit(controller::random_foraging_controller& controller) {
  controller.fsm()->accept(*this);
  controller.block(nullptr);
  ER_NOM("random_foraging_controller: %s dropped block%d in nest",
         controller.GetId().c_str(), m_block->id());
} /* visit() */

void nest_block_drop::visit(controller::memory_foraging_controller& controller) {
  controller.fsm()->accept(*this);
  controller.block(nullptr);
  ER_NOM("memory_foraging_controller: %s dropped block%d in nest",
         controller.GetId().c_str(), m_block->id());
} /* visit() */

void nest_block_drop::visit(fsm::random_foraging_fsm& fsm) {
  ER_NOM("random_foraging_fsm: register nest_block_drop event");
  fsm.inject_event(controller::foraging_signal::BLOCK_DROP,
                   state_machine::event_type::NORMAL);
} /* visit() */

void nest_block_drop::visit(fsm::memory_foraging_fsm& fsm) {
  ER_NOM("memory_foraging_fsm: register nest_block_drop event");
  fsm.inject_event(controller::foraging_signal::BLOCK_DROP,
                   state_machine::event_type::NORMAL);
} /* visit() */

NS_END(events, fordyca);
