/**
 * @file stateful_foraging_fsm.cpp
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
#include "fordyca/fsm/depth0/stateful_foraging_fsm.hpp"
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/params/fsm_params.hpp"

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
    controller::saa_subsystem* const saa,
    representation::perceived_arena_map* const map)
    : base_foraging_fsm(server, saa, ST_MAX_STATES),
      HFSM_CONSTRUCT_STATE(leaving_nest, &start),
      HFSM_CONSTRUCT_STATE(transport_to_nest, &start),
      entry_wait_for_signal(),
      entry_transport_to_nest(),
      entry_leaving_nest(),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_block, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_pickup, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      m_block_fsm(params, server, saa, map),
      mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
                   HFSM_STATE_MAP_ENTRY_EX(&acquire_block),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_pickup,
                                               nullptr,
                                               &entry_wait_for_signal,
                                               nullptr),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&transport_to_nest,
                                               nullptr,
                                               &entry_transport_to_nest,
                                               nullptr),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&leaving_nest,
                                               nullptr,
                                               &entry_leaving_nest,
                                               nullptr),
                   HFSM_STATE_MAP_ENTRY_EX(&finished)} {
  hfsm::change_parent(ST_LEAVING_NEST, &start);
}

HFSM_STATE_DEFINE(stateful_foraging_fsm, start, state_machine::event_data) {
  /* first time running FSM */
  if (state_machine::event_type::NORMAL == data->type()) {
    internal_event(ST_ACQUIRE_BLOCK);
    return controller::foraging_signal::HANDLED;
  }
  if (state_machine::event_type::CHILD == data->type()) {
    if (controller::foraging_signal::BLOCK_DROP == data->signal()) {
      internal_event(ST_LEAVING_NEST);
      return controller::foraging_signal::HANDLED;
    } else if (controller::foraging_signal::LEFT_NEST == data->signal()) {
      m_task_running = false;
      internal_event(ST_ACQUIRE_BLOCK);
      return controller::foraging_signal::HANDLED;
    }
  }
  ER_FATAL_SENTINEL("FATAL: Unhandled signal");
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE_ND(stateful_foraging_fsm, acquire_block) {
  if (m_block_fsm.task_finished()) {
    actuators()->differential_drive().stop();
    internal_event(ST_WAIT_FOR_PICKUP);
  } else {
    m_block_fsm.task_execute();
  }
  return controller::foraging_signal::HANDLED;
}
HFSM_STATE_DEFINE(stateful_foraging_fsm,
                  wait_for_pickup,
                  state_machine::event_data) {
  if (controller::foraging_signal::BLOCK_PICKUP == data->signal()) {
    m_block_fsm.task_reset();
    m_pickup_count = 0;
    internal_event(ST_TRANSPORT_TO_NEST);
  }
  ++m_pickup_count;
  if (m_pickup_count >= kPICKUP_TIMEOUT) {
    m_pickup_count = 0;
    m_block_fsm.task_reset();
    internal_event(ST_ACQUIRE_BLOCK);
  }
  return controller::foraging_signal::HANDLED;
}

__rcsw_const FSM_STATE_DEFINE_ND(stateful_foraging_fsm, finished) {
  return controller::foraging_signal::HANDLED;
}

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
FSM_WRAPPER_DEFINE(bool,
                   stateful_foraging_fsm,
                   is_exploring_for_goal,
                   m_block_fsm);

FSM_WRAPPER_DEFINE(bool,
                   stateful_foraging_fsm,
                   is_vectoring_to_goal,
                   m_block_fsm);

acquisition_goal_type stateful_foraging_fsm::acquisition_goal(void) const {
  if (ST_ACQUIRE_BLOCK == current_state() ||
      ST_WAIT_FOR_PICKUP == current_state()) {
    return acquisition_goal_type::kBlock;
  }
  return acquisition_goal_type::kNone;
} /* acquisition_goal() */

bool stateful_foraging_fsm::goal_acquired(void) const {
  return current_state() == ST_WAIT_FOR_PICKUP;
} /* goal_acquired() */

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

transport_goal_type stateful_foraging_fsm::block_transport_goal(void) const {
  if (ST_TRANSPORT_TO_NEST == current_state()) {
    return transport_goal_type::kNest;
  }
  return transport_goal_type::kNone;
} /* acquisition_goal() */

NS_END(depth0, fsm, fordyca);
