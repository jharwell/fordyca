/**
 * \file crw_fsm.cpp
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
#include "fordyca/fsm/depth0/crw_fsm.hpp"

#include "cosm/robots/footbot/footbot_actuation_subsystem.hpp"
#include "cosm/robots/footbot/footbot_saa_subsystem2D.hpp"
#include "cosm/robots/footbot/footbot_sensing_subsystem.hpp"

#include "fordyca/fsm/expstrat/crw.hpp"
#include "fordyca/fsm/foraging_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth0);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
crw_fsm::crw_fsm(crfootbot::footbot_saa_subsystem2D* const saa,
                 std::unique_ptr<fsm::expstrat::foraging_expstrat> exp_behavior,
                 rmath::rng* rng)
    : util_hfsm(saa, rng, ekST_MAX_STATES),
      ER_CLIENT_INIT("fordyca.fsm.depth0.crw"),
      HFSM_CONSTRUCT_STATE(transport_to_nest, &start),
      HFSM_CONSTRUCT_STATE(leaving_nest, &start),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_block, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_block_pickup, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_block_drop, hfsm::top_state()),
      HFSM_DEFINE_STATE_MAP(mc_state_map,
                            HFSM_STATE_MAP_ENTRY_EX(&start),
                            HFSM_STATE_MAP_ENTRY_EX(&acquire_block),
                            HFSM_STATE_MAP_ENTRY_EX_ALL(&transport_to_nest,
                                                        nullptr,
                                                        &entry_transport_to_nest,
                                                        &exit_transport_to_nest),
                            HFSM_STATE_MAP_ENTRY_EX_ALL(&leaving_nest,
                                                        nullptr,
                                                        &entry_leaving_nest,
                                                        nullptr),
                            HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_pickup,
                                                        nullptr,
                                                        &entry_wait_for_signal,
                                                        nullptr),
                            HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_drop,
                                                        nullptr,
                                                        &entry_wait_for_signal,
                                                        nullptr)),
      m_explore_fsm(saa,
                    std::move(exp_behavior),
                    rng,
                    std::bind(&crw_fsm::block_detected, this)) {}

/*******************************************************************************
 * States
 ******************************************************************************/
HFSM_STATE_DEFINE(crw_fsm, start, rpfsm::event_data* data) {
  /* first time running FSM */
  if (rpfsm::event_type::ekNORMAL == data->type()) {
    internal_event(ekST_ACQUIRE_BLOCK);
    return fsm::foraging_signal::ekHANDLED;
  }
  if (rpfsm::event_type::ekCHILD == data->type()) {
    if (fsm::foraging_signal::ekLEFT_NEST == data->signal()) {
      m_explore_fsm.task_start(nullptr);
      internal_event(ekST_ACQUIRE_BLOCK);
      return fsm::foraging_signal::ekHANDLED;
    } else if (fsm::foraging_signal::ekENTERED_NEST == data->signal()) {
      internal_event(ekST_WAIT_FOR_BLOCK_DROP);
      return fsm::foraging_signal::ekHANDLED;
    }
  }
  ER_FATAL_SENTINEL("Unhandled signal %d", data->signal());
  return fsm::foraging_signal::ekHANDLED;
}

HFSM_STATE_DEFINE_ND(crw_fsm, acquire_block) {
  if (m_explore_fsm.task_finished()) {
    internal_event(ekST_WAIT_FOR_BLOCK_PICKUP);
  } else {
    m_explore_fsm.task_execute();
  }
  return fsm::foraging_signal::ekHANDLED;
}

HFSM_STATE_DEFINE(crw_fsm, wait_for_block_pickup, rpfsm::event_data* data) {
  if (fsm::foraging_signal::ekBLOCK_PICKUP == data->signal()) {
    m_explore_fsm.task_reset();
    ER_INFO("Block pickup signal received");
    internal_event(ekST_TRANSPORT_TO_NEST);
  } else if (fsm::foraging_signal::ekBLOCK_VANISHED == data->signal()) {
    m_explore_fsm.task_reset();
    internal_event(ekST_ACQUIRE_BLOCK);
  }
  return fsm::foraging_signal::ekHANDLED;
}

HFSM_STATE_DEFINE(crw_fsm, wait_for_block_drop, rpfsm::event_data* data) {
  if (fsm::foraging_signal::ekBLOCK_DROP == data->signal()) {
    m_explore_fsm.task_reset();
    ER_INFO("Block drop signal received");
    internal_event(ekST_LEAVING_NEST);
  }
  return fsm::foraging_signal::ekHANDLED;
}

/*******************************************************************************
 * Metrics
 ******************************************************************************/
crw_fsm::exp_status crw_fsm::is_exploring_for_goal(void) const {
  return std::make_pair(current_state() == ekST_ACQUIRE_BLOCK, true);
} /* is_exploring_for_goal() */

bool crw_fsm::goal_acquired(void) const {
  if (foraging_acq_goal::type::ekBLOCK == acquisition_goal()) {
    return current_state() == ekST_WAIT_FOR_BLOCK_PICKUP;
  } else if (foraging_transport_goal::type::ekNEST == block_transport_goal()) {
    return current_state() == ekST_WAIT_FOR_BLOCK_DROP;
  }
  return false;
} /* goal_acquired() */

rmath::vector2u crw_fsm::acquisition_loc(void) const {
  return saa()->sensing()->discrete_position();
} /* acquisition_loc() */

rmath::vector2u crw_fsm::current_explore_loc(void) const {
  return saa()->sensing()->discrete_position();
} /* current_explore_loc() */

rmath::vector2u crw_fsm::current_vector_loc(void) const {
  ER_FATAL_SENTINEL("CRW_FSM current vector location undefined");
  return saa()->sensing()->discrete_position();
} /* current_vector_loc() */

rtypes::type_uuid crw_fsm::entity_acquired_id(void) const {
  /* CRW FSM has no concept of state, so it doesn't know what it has acquired */
  return rtypes::constants::kNoUUID;
} /* entity_acquired_id() */

/*******************************************************************************
 * Collision Metrics
 ******************************************************************************/
bool crw_fsm::in_collision_avoidance(void) const {
  return (m_explore_fsm.task_running() &&
          m_explore_fsm.in_collision_avoidance()) ||
         cfsm::util_hfsm::in_collision_avoidance();
} /* in_collision_avoidance() */

bool crw_fsm::entered_collision_avoidance(void) const {
  return (m_explore_fsm.task_running() &&
          m_explore_fsm.entered_collision_avoidance()) ||
         cfsm::util_hfsm::entered_collision_avoidance();
} /* entered_collision_avoidance() */

bool crw_fsm::exited_collision_avoidance(void) const {
  return (m_explore_fsm.task_running() &&
          m_explore_fsm.exited_collision_avoidance()) ||
         cfsm::util_hfsm::exited_collision_avoidance();
} /* exited_collision_avoidance() */

rtypes::timestep crw_fsm::collision_avoidance_duration(void) const {
  if (m_explore_fsm.task_running()) {
    return m_explore_fsm.collision_avoidance_duration();
  } else {
    return cfsm::util_hfsm::collision_avoidance_duration();
  }
} /* collision_avoidance_duration() */

rmath::vector2u crw_fsm::avoidance_loc(void) const {
  return saa()->sensing()->discrete_position();
} /* avoidance_loc() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void crw_fsm::init(void) {
  cfsm::util_hfsm::init();
  m_explore_fsm.init();
} /* init() */

void crw_fsm::run(void) {
  inject_event(fsm::foraging_signal::ekRUN, rpfsm::event_type::ekNORMAL);
} /* run() */

bool crw_fsm::block_detected(void) const {
  return saa()->sensing()->sensor<chal::sensors::ground_sensor>()->detect(
      "block");
} /* block_detected() */

foraging_transport_goal::type crw_fsm::block_transport_goal(void) const {
  if (ekST_TRANSPORT_TO_NEST == current_state() ||
      ekST_WAIT_FOR_BLOCK_DROP == current_state()) {
    return foraging_transport_goal::type::ekNEST;
  }
  return foraging_transport_goal::type::ekNONE;
} /* block_transport_goal() */

cfsm::metrics::goal_acq_metrics::goal_type crw_fsm::acquisition_goal(void) const {
  if (ekST_ACQUIRE_BLOCK == current_state() ||
      ekST_WAIT_FOR_BLOCK_PICKUP == current_state()) {
    return cfsm::metrics::goal_acq_metrics::goal_type(
        foraging_acq_goal::type::ekBLOCK);
  }
  return cfsm::metrics::goal_acq_metrics::goal_type(
      foraging_acq_goal::type::ekNONE);
} /* block_transport_goal() */

NS_END(depth0, fsm, fordyca);
