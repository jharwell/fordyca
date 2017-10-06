/**
 * @file block_to_cache_fsm.cpp
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
#include "fordyca/fsm/block_to_cache_fsm.hpp"
#include "fordyca/controller/foraging_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_to_cache_fsm::block_to_cache_fsm(
    const struct params::fsm_params* params,
    const std::shared_ptr<rcppsw::common::er_server>& server,
    const std::shared_ptr<controller::sensor_manager>& sensors,
    const std::shared_ptr<controller::actuator_manager>& actuators,
    const std::shared_ptr<const representation::perceived_arena_map>& map) :
    base_foraging_fsm(server, sensors, actuators, ST_MAX_STATES),
    HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(acquire_free_block, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(transport_to_cache, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
    m_block_fsm(params, server, sensors, actuators, map),
    m_cache_fsm(params, server, sensors, actuators, map) {}

HFSM_STATE_DEFINE(block_to_cache_fsm, start, state_machine::event_data) {
  if (data) {
    ER_ASSERT(controller::foraging_signal::ACQUIRE_FREE_BLOCK == data->signal(),
              "FATAL: Unhandled signal type");
        internal_event(ST_ACQUIRE_FREE_BLOCK);
  }
  return controller::foraging_signal::HANDLED;
}
HFSM_STATE_DEFINE(block_to_cache_fsm, acquire_free_block, state_machine::event_data) {
  ER_ASSERT(state_machine::event_type::NORMAL == data->type(), "Bad event type");
  if (m_block_fsm.task_finished()) {
    if (controller::foraging_signal::BLOCK_PICKUP == data->signal()) {
      m_block_fsm.task_reset();
      internal_event(ST_TRANSPORT_TO_CACHE);
    }
  } else {
    m_block_fsm.task_execute();
  }
  return controller::foraging_signal::HANDLED;
}
HFSM_STATE_DEFINE(block_to_cache_fsm, transport_to_cache, state_machine::event_data) {
  ER_ASSERT(state_machine::event_type::NORMAL == data->type(), "Bad event type");

  if (m_cache_fsm.task_finished()) {
    if (controller::foraging_signal::BLOCK_PICKUP == data->signal()) {
      m_cache_fsm.task_reset();
      internal_event(ST_TRANSPORT_TO_CACHE);
    }
  }
  m_cache_fsm.task_execute();
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE(block_to_cache_fsm, finished, state_machine::event_data) {
  return controller::foraging_signal::HANDLED;
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void block_to_cache_fsm::init(void) {
  base_foraging_fsm::init();
  m_cache_fsm.task_reset();
  m_block_fsm.task_reset();
} /* init() */

void block_to_cache_fsm::task_start(const rcppsw::task_allocation::taskable_argument* const arg) {
  inject_event(controller::foraging_signal::ACQUIRE_FREE_BLOCK,
               state_machine::event_type::NORMAL);
}

void block_to_cache_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* task_execute() */



NS_END(fsm, fordyca);
