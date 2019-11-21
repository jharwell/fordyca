/**
 * \file free_block_to_nest_fsm.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/fsm/depth0/free_block_to_nest_fsm.hpp"

#include "fordyca/fsm/expstrat/foraging_expstrat.hpp"
#include "fordyca/fsm/foraging_goal_type.hpp"
#include "fordyca/fsm/foraging_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth0);

using goal_type = cfmetrics::goal_acq_metrics::goal_type;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
free_block_to_nest_fsm::free_block_to_nest_fsm(
    const fsm_ro_params* c_params,
    crfootbot::footbot_saa_subsystem* saa,
    std::unique_ptr<fsm::expstrat::foraging_expstrat> exp_behavior,
    rmath::rng* rng)
    : util_hfsm(saa, rng, ekST_MAX_STATES),
      ER_CLIENT_INIT("fordyca.fsm.depth0.free_block_to_nest"),
      HFSM_CONSTRUCT_STATE(leaving_nest, &start),
      HFSM_CONSTRUCT_STATE(transport_to_nest, &start),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_block, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_pickup, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_drop, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      m_block_fsm(c_params, saa, std::move(exp_behavior), rng),
      mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
                   HFSM_STATE_MAP_ENTRY_EX(&acquire_block),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_pickup,
                                               nullptr,
                                               &entry_wait_for_signal,
                                               nullptr),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_drop,
                                               nullptr,
                                               &entry_wait_for_signal,
                                               nullptr),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&transport_to_nest,
                                               nullptr,
                                               &entry_transport_to_nest,
                                               &exit_transport_to_nest),
                   HFSM_STATE_MAP_ENTRY_EX(&finished)} {}

HFSM_STATE_DEFINE(free_block_to_nest_fsm, start, rpfsm::event_data* data) {
  /* first time running FSM */
  if (rpfsm::event_type::ekNORMAL == data->type()) {
    internal_event(ekST_ACQUIRE_BLOCK);
    return fsm::foraging_signal::ekHANDLED;
  }
  if (rpfsm::event_type::ekCHILD == data->type()) {
    if (fsm::foraging_signal::ekENTERED_NEST == data->signal()) {
      internal_event(ekST_WAIT_FOR_DROP);
      return fsm::foraging_signal::ekHANDLED;
    }
  }
  ER_FATAL_SENTINEL("Unhandled signal %d", data->signal());
  return fsm::foraging_signal::ekHANDLED;
}

HFSM_STATE_DEFINE_ND(free_block_to_nest_fsm, acquire_block) {
  if (m_block_fsm.task_finished()) {
    internal_event(ekST_WAIT_FOR_PICKUP);
  } else {
    m_block_fsm.task_execute();
  }
  return fsm::foraging_signal::ekHANDLED;
}
HFSM_STATE_DEFINE(free_block_to_nest_fsm,
                  wait_for_pickup,
                  rpfsm::event_data* data) {
  /**
   * It is possible that robots can be waiting indefinitely for a block
   * pickup signal that will never come once a block has been acquired if they
   * "detect" a block by sprawling across multiple blocks (i.e. all ground
   * sensors did not detect the same block). It is also possible that a robot
   * serving a penalty for a block pickup will have the block taken by a
   * different robot.
   *
   * In both cases, treat the block as vanished and try again.
   */
  if (fsm::foraging_signal::ekBLOCK_PICKUP == data->signal()) {
    m_block_fsm.task_reset();
    internal_event(ekST_TRANSPORT_TO_NEST);
  } else if (fsm::foraging_signal::ekBLOCK_VANISHED == data->signal()) {
    m_block_fsm.task_reset();
    internal_event(ekST_ACQUIRE_BLOCK);
  }
  return fsm::foraging_signal::ekHANDLED;
}
HFSM_STATE_DEFINE(free_block_to_nest_fsm,
                  wait_for_drop,
                  rpfsm::event_data* data) {
  if (fsm::foraging_signal::ekBLOCK_DROP == data->signal()) {
    m_block_fsm.task_reset();
    internal_event(ekST_FINISHED);
  }
  return fsm::foraging_signal::ekHANDLED;
}

RCSW_CONST FSM_STATE_DEFINE_ND(free_block_to_nest_fsm, finished) {
  return fsm::foraging_signal::ekHANDLED;
}

/*******************************************************************************
 * Collision Metrics
 ******************************************************************************/
bool free_block_to_nest_fsm::in_collision_avoidance(void) const {
  return (m_block_fsm.task_running() && m_block_fsm.in_collision_avoidance()) ||
         cfsm::util_hfsm::in_collision_avoidance();
} /* in_collision_avoidance() */

bool free_block_to_nest_fsm::entered_collision_avoidance(void) const {
  return (m_block_fsm.task_running() &&
          m_block_fsm.entered_collision_avoidance()) ||
         cfsm::util_hfsm::entered_collision_avoidance();
} /* entered_collision_avoidance() */

bool free_block_to_nest_fsm::exited_collision_avoidance(void) const {
  return (m_block_fsm.task_running() &&
          m_block_fsm.exited_collision_avoidance()) ||
         cfsm::util_hfsm::exited_collision_avoidance();
} /* exited_collision_avoidance() */

rtypes::timestep free_block_to_nest_fsm::collision_avoidance_duration(void) const {
  if (m_block_fsm.task_running()) {
    return m_block_fsm.collision_avoidance_duration();
  } else {
    return cfsm::util_hfsm::collision_avoidance_duration();
  }
} /* collision_avoidance_duration() */

rmath::vector2u free_block_to_nest_fsm::avoidance_loc(void) const {
  if (m_block_fsm.task_running()) {
    return m_block_fsm.avoidance_loc();
  } else {
    return cfsm::util_hfsm::avoidance_loc();
  }
} /* collision_avoidance_duration() */

/*******************************************************************************
 * Goal Acquisition Metrics
 ******************************************************************************/
RCPPSW_WRAP_OVERRIDE_DEF(free_block_to_nest_fsm,
                         is_exploring_for_goal,
                         m_block_fsm,
                         const);
RCPPSW_WRAP_OVERRIDE_DEF(free_block_to_nest_fsm,
                         is_vectoring_to_goal,
                         m_block_fsm,
                         const);

RCPPSW_WRAP_OVERRIDE_DEF(free_block_to_nest_fsm,
                         acquisition_loc,
                         m_block_fsm,
                         const);

RCPPSW_WRAP_OVERRIDE_DEF(free_block_to_nest_fsm,
                         current_explore_loc,
                         m_block_fsm,
                         const);
RCPPSW_WRAP_OVERRIDE_DEF(free_block_to_nest_fsm,
                         current_vector_loc,
                         m_block_fsm,
                         const);

goal_type free_block_to_nest_fsm::acquisition_goal(void) const {
  if (ekST_ACQUIRE_BLOCK == current_state() ||
      ekST_WAIT_FOR_PICKUP == current_state()) {
    return goal_type(foraging_acq_goal::type::ekBLOCK);
  }
  return goal_type(foraging_acq_goal::type::ekNONE);
} /* acquisition_goal() */

bool free_block_to_nest_fsm::goal_acquired(void) const {
  if (foraging_acq_goal::type::ekBLOCK == acquisition_goal()) {
    return current_state() == ekST_WAIT_FOR_PICKUP;
  } else if (foraging_transport_goal::type::ekNEST == block_transport_goal()) {
    return current_state() == ekST_WAIT_FOR_DROP;
  }
  return false;
} /* goal_acquired() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void free_block_to_nest_fsm::init(void) {
  cfsm::util_hfsm::init();
  m_block_fsm.task_reset();
} /* init() */

void free_block_to_nest_fsm::task_execute(void) {
  inject_event(fsm::foraging_signal::ekRUN, rpfsm::event_type::ekNORMAL);
} /* task_execute() */

foraging_transport_goal::type free_block_to_nest_fsm::block_transport_goal(
    void) const {
  if (ekST_TRANSPORT_TO_NEST == current_state() ||
      ekST_WAIT_FOR_DROP == current_state()) {
    return foraging_transport_goal::type::ekNEST;
  }
  return foraging_transport_goal::type::ekNONE;
} /* acquisition_goal() */

NS_END(depth0, fsm, fordyca);
