/**
 * @file foraging_fsm.cpp
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
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/fsm/depth0/stateful_foraging_fsm.hpp"
#include "fordyca/controller/depth1/foraging_sensors.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth0);
namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
stateful_foraging_fsm::stateful_foraging_fsm(
    const struct params::fsm_params* params,
    const std::shared_ptr<rcppsw::er::server>& server,
    const std::shared_ptr<controller::depth1::foraging_sensors>& sensors,
    const std::shared_ptr<controller::actuator_manager>& actuators,
    const std::shared_ptr<const representation::perceived_arena_map>& map) :
    base_foraging_fsm(server,
                      std::static_pointer_cast<controller::base_foraging_sensors>(sensors),
                      actuators, ST_MAX_STATES),
    HFSM_CONSTRUCT_STATE(leaving_nest, &start),
    entry_leaving_nest(),
    HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(block_to_nest, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
    m_task_running(false),
    m_sensors(sensors),
    m_block_fsm(params, server,
                std::static_pointer_cast<controller::depth1::foraging_sensors>(sensors),
                actuators, map),
    mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
      HFSM_STATE_MAP_ENTRY_EX(&block_to_nest),
      HFSM_STATE_MAP_ENTRY_EX_ALL(&leaving_nest, NULL,
                                  &entry_leaving_nest, NULL),
      HFSM_STATE_MAP_ENTRY_EX(&finished)} {
  hfsm::change_parent(ST_LEAVING_NEST, &start);
    }

HFSM_STATE_DEFINE(stateful_foraging_fsm, start, state_machine::event_data) {
  /* first time running FSM */
  if (state_machine::event_type::NORMAL == data->type()) {
    ER_NOM("Starting foraging");
    internal_event(ST_ACQUIRE_FREE_BLOCK);
    return controller::foraging_signal::HANDLED;
  }
  if (state_machine::event_type::CHILD == data->type()) {
    if (controller::foraging_signal::BLOCK_DROP == data->signal()) {
      internal_event(ST_LEAVING_NEST);
      return controller::foraging_signal::HANDLED;
    } else if (controller::foraging_signal::LEFT_NEST == data->signal()) {
      m_task_running = false;
      internal_event(ST_ACQUIRE_FREE_BLOCK);
      return controller::foraging_signal::HANDLED;
    }
  }
  ER_ASSERT(0, "FATAL: Unhandled signal");
  return controller::foraging_signal::HANDLED;
}
  HFSM_STATE_DEFINE(stateful_foraging_fsm, block_to_nest, state_machine::event_data) {
  ER_ASSERT(state_machine::event_type::NORMAL == data->type(), "Bad event type");

  /* first time running FSM; transitioned from START state */
  if (!this->task_running()) {
    tasks::foraging_signal_argument a(controller::foraging_signal::ACQUIRE_FREE_BLOCK);
    m_block_fsm.task_start(&a);
    m_task_running = true;
    return controller::foraging_signal::HANDLED;
  }

  /*
   * Wait in the finished state until the controller tells us we have dropped a
   * block.
   */
  if (m_block_fsm.task_finished()) {
    internal_event(ST_FINISHED);
  }

  /*
   * If we have gotten the block pickup signal from the controller, relay it to
   * the acquire_block sub-FSM so that it will start returning to the nest.
   */
  if (controller::foraging_signal::BLOCK_PICKUP == data->signal()) {
    m_block_fsm.inject_event(data->signal(), data->type());
  } else if (controller::foraging_signal::BLOCK_DROP == data->signal()) {
    m_block_fsm.inject_event(data->signal(), data->type());
  }
  m_block_fsm.task_execute();
  return controller::foraging_signal::HANDLED;
}

__const FSM_STATE_DEFINE_ND(stateful_foraging_fsm, finished) {
  return controller::foraging_signal::HANDLED;
}

/*******************************************************************************
 * Base Diagnostics
 ******************************************************************************/
bool stateful_foraging_fsm::is_exploring_for_block(void) const {
  return m_block_fsm.is_exploring_for_block();
} /* is_exploring_for_block() */

bool stateful_foraging_fsm::is_avoiding_collision(void) const {
  return m_block_fsm.is_avoiding_collision();
} /* is_avoiding_collision() */

bool stateful_foraging_fsm::is_transporting_to_nest(void) const {
  return m_block_fsm.is_transporting_to_nest();
} /* is_transporting_to_nest() */

/*******************************************************************************
 * Depth0 Diagnostics
 ******************************************************************************/
bool stateful_foraging_fsm::is_acquiring_block(void) const {
  return m_block_fsm.is_acquiring_block();
} /* is_acquiring_block() */

bool stateful_foraging_fsm::is_vectoring_to_block(void) const {
  return m_block_fsm.is_vectoring_to_block();
} /* is_vectoring_to_block() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void stateful_foraging_fsm::init(void) {
  base_foraging_fsm::init();
  m_block_fsm.task_reset();
  m_task_running = false;
} /* init() */

void stateful_foraging_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* task_execute() */

bool stateful_foraging_fsm::block_acquired(void) const {
  return m_block_fsm.block_acquired();
} /* block_acquired() */


NS_END(depth0, fsm, fordyca);
