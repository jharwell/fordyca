/**
 * @file block_to_nest_fsm.cpp
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
#include "fordyca/fsm/block_to_nest_fsm.hpp"
#include "fordyca/controller/foraging_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_to_nest_fsm::block_to_nest_fsm(
    const struct params::fsm_params* params,
    const std::shared_ptr<rcppsw::common::er_server>& server,
    const std::shared_ptr<controller::sensor_manager>& sensors,
    const std::shared_ptr<controller::actuator_manager>& actuators,
    const std::shared_ptr<const representation::perceived_arena_map>& map) :
    base_foraging_fsm(server, sensors, actuators, ST_MAX_STATES),
    HFSM_CONSTRUCT_STATE(return_to_nest, hfsm::top_state()),
    entry_return_to_nest(),
    HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(acquire_free_block, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(acquire_cached_block, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
    m_block_fsm(params, server, sensors, actuators, map),
    m_cache_fsm(params, server, sensors, actuators, map),
    mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
        HFSM_STATE_MAP_ENTRY_EX(&acquire_free_block),
        HFSM_STATE_MAP_ENTRY_EX(&acquire_cached_block),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&return_to_nest, NULL,
                                    &entry_return_to_nest, NULL),
        HFSM_STATE_MAP_ENTRY_EX(&finished)} {
  hfsm::change_parent(ST_RETURN_TO_NEST, &start);
    }

HFSM_STATE_DEFINE(block_to_nest_fsm, start, state_machine::event_data) {
  if (state_machine::event_type::NORMAL == data->type()) {
    if (controller::foraging_signal::ACQUIRE_FREE_BLOCK == data->signal()) {
      internal_event(ST_ACQUIRE_FREE_BLOCK);
      return controller::foraging_signal::HANDLED;
    } else if (controller::foraging_signal::ACQUIRE_CACHED_BLOCK == data->signal()) {
      internal_event(ST_ACQUIRE_CACHED_BLOCK);
      return controller::foraging_signal::HANDLED;
    }
  } else if (state_machine::event_type::CHILD == data->type()) {
    if (controller::foraging_signal::BLOCK_DROP == data->signal()) {
      internal_event(ST_FINISHED);
      return controller::foraging_signal::HANDLED;
    }
  }
  return controller::foraging_signal::HANDLED;
}
HFSM_STATE_DEFINE(block_to_nest_fsm, acquire_free_block, state_machine::event_data) {
  ER_ASSERT(state_machine::event_type::NORMAL == data->type(), "Bad event type");
  /*
   * We wait until an upper FSM/controller tells us a free block has been
   * successfully picked up before we start moving back towards the nest.
   */
  if (controller::foraging_signal::BLOCK_PICKUP == data->signal()) {
    m_block_fsm.task_reset();
    internal_event(ST_RETURN_TO_NEST);
  } else {
    m_block_fsm.task_execute();
  }
  return controller::foraging_signal::HANDLED;
}
HFSM_STATE_DEFINE(block_to_nest_fsm, acquire_cached_block, state_machine::event_data) {
  ER_ASSERT(state_machine::event_type::NORMAL == data->type(), "Bad event type");
  /*
   * We wait in the finished state until an upper FSM/controller tells us a
   * block has been successfully picked up from a cache before we start moving
   * back towards the nest.
   */
  if (m_cache_fsm.task_finished()) {
    if (controller::foraging_signal::BLOCK_PICKUP == data->signal()) {
      m_cache_fsm.task_reset();
      internal_event(ST_RETURN_TO_NEST);
    }
  } else {
    m_cache_fsm.task_execute();
  }
  return controller::foraging_signal::HANDLED;
}
HFSM_STATE_DEFINE(block_to_nest_fsm, finished, state_machine::no_event_data) {
  return controller::foraging_signal::HANDLED;
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void block_to_nest_fsm::init(void) {
  base_foraging_fsm::init();
  m_cache_fsm.task_reset();
  m_block_fsm.task_reset();
} /* init() */

void block_to_nest_fsm::task_start(const rcppsw::task_allocation::taskable_argument* const arg) {
  const tasks::foraging_signal_argument* const a =
      dynamic_cast<const tasks::foraging_signal_argument* const>(arg);
  ER_ASSERT(a, "FATAL: bad argument passed");
  inject_event(
      a->signal(), state_machine::event_type::NORMAL);
}

void block_to_nest_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* task_execute() */

bool block_to_nest_fsm::is_searching_for_block(void) const {
  return m_block_fsm.is_searching_for_block();
}

bool block_to_nest_fsm::is_searching_for_cache(void) const {
  return m_cache_fsm.is_searching_for_cache();
}

bool block_to_nest_fsm::is_exploring(void) const {
  return m_block_fsm.is_exploring() || m_cache_fsm.is_exploring();
}
bool block_to_nest_fsm::is_vectoring(void) const {
  return m_block_fsm.is_vectoring() || m_cache_fsm.is_exploring();
}

bool block_to_nest_fsm::is_avoiding_collision(void) const {
  return m_block_fsm.is_avoiding_collision() ||
      m_cache_fsm.is_avoiding_collision();
}
bool block_to_nest_fsm::is_transporting_to_nest(void) const {
  return current_state() == ST_RETURN_TO_NEST;
}

NS_END(fsm, fordyca);
