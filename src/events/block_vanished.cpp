/**
 * @file block_vanished.cpp
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
n * FORDYCA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * FORDYCA.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/events/block_vanished.hpp"
#include "fordyca/controller/depth0/stateless_foraging_controller.hpp"
#include "fordyca/controller/depth0/stateful_foraging_controller.hpp"
#include "fordyca/controller/depth1/foraging_controller.hpp"
#include "fordyca/controller/depth2/foraging_controller.hpp"
#include "fordyca/fsm/depth1/block_to_goal_fsm.hpp"
#include "fordyca/fsm/depth0/stateless_foraging_fsm.hpp"
#include "fordyca/fsm/depth0/stateful_foraging_fsm.hpp"
#include "fordyca/tasks/depth0/generalist.hpp"
#include "fordyca/tasks/depth1/harvester.hpp"
#include "fordyca/tasks/depth2/cache_starter.hpp"
#include "fordyca/tasks/depth2/cache_finisher.hpp"
#include "fordyca/fsm/depth2/block_to_cache_site_fsm.hpp"
#include "fordyca/fsm/depth2/block_to_new_cache_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_vanished::block_vanished(std::shared_ptr<rcppsw::er::server> server,
                               uint block_id)
    : client(server), m_block_id(block_id) {
  client::insmod("block_vanished",
                 rcppsw::er::er_lvl::DIAG,
                 rcppsw::er::er_lvl::NOM);
}

/*******************************************************************************
 * Depth0 Foraging
 ******************************************************************************/
void block_vanished::visit(controller::depth0::stateless_foraging_controller& controller) {
  ER_NOM("%s abort pickup: block%d vanished",
         controller.GetId().c_str(),
         m_block_id);
  controller.fsm()->accept(*this);
} /* visit() */

void block_vanished::visit(controller::depth0::stateful_foraging_controller& controller) {
  ER_NOM("%s abort pickup: block%d vanished",
         controller.GetId().c_str(),
         m_block_id);
  dynamic_cast<tasks::depth0::generalist*>(controller.current_task())->accept(*this);
} /* visit() */

void block_vanished::visit(tasks::depth0::generalist& task) {
  static_cast<fsm::depth0::stateful_foraging_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

void block_vanished::visit(fsm::depth0::stateless_foraging_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_VANISHED,
                   state_machine::event_type::NORMAL);
} /* visit() */

void block_vanished::visit(fsm::depth0::stateful_foraging_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_VANISHED,
                   state_machine::event_type::NORMAL);
} /* visit() */

/*******************************************************************************
 * Depth1 Foraging
 ******************************************************************************/
void block_vanished::visit(controller::depth1::foraging_controller& controller) {
  ER_NOM("%s abort pickup: block%d vanished",
         controller.GetId().c_str(),
         m_block_id);
  auto *task = dynamic_cast<tasks::depth1::harvester*>(controller.current_task());
  ER_ASSERT(nullptr != task, "FATAL: Non-harvester task triggered block vanished event");
  task->accept(*this);
} /* visit() */

void block_vanished::visit(tasks::depth1::harvester& task) {
  static_cast<fsm::depth1::block_to_goal_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

void block_vanished::visit(fsm::depth1::block_to_goal_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_VANISHED,
                   state_machine::event_type::NORMAL);
} /* visit() */

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void block_vanished::visit(controller::depth2::foraging_controller& controller) {
  ER_NOM("%s abort pickup/drop from/in block: block%d vanished",
         controller.GetId().c_str(),
         m_block_id);
  dynamic_cast<tasks::free_block_interactor*>(controller.current_task())
      ->accept(*this);
} /* visit() */

void block_vanished::visit(tasks::depth2::cache_starter& task) {
  static_cast<fsm::depth1::block_to_goal_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

void block_vanished::visit(tasks::depth2::cache_finisher& task) {
  static_cast<fsm::depth1::block_to_goal_fsm*>(task.mechanism())->accept(*this);
} /* visit() */


NS_END(events, fordyca);
