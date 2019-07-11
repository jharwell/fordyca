/**
 * @file cached_block_to_nest_fsm.cpp
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
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth1);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
cached_block_to_nest_fsm::cached_block_to_nest_fsm(
    const controller::cache_sel_matrix* sel_matrix,
    controller::saa_subsystem* const saa,
    ds::dpo_store* const store,
    std::unique_ptr<expstrat::base_expstrat> exp_behavior)
    : base_foraging_fsm(saa, ekST_MAX_STATES),
      ER_CLIENT_INIT("fordyca.fsm.depth1.cached_block_to_nest"),
      HFSM_CONSTRUCT_STATE(transport_to_nest, &start),
      HFSM_CONSTRUCT_STATE(leaving_nest, &start),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_block, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_pickup, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_drop, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      m_cache_fsm(sel_matrix, saa, store, std::move(exp_behavior), true),
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
                                               nullptr),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&leaving_nest,
                                               nullptr,
                                               &entry_leaving_nest,
                                               nullptr),
                   HFSM_STATE_MAP_ENTRY_EX(&finished)} {}

HFSM_STATE_DEFINE(cached_block_to_nest_fsm, start, rpfsm::event_data* data) {
  if (rpfsm::event_type::ekNORMAL == data->type()) {
    internal_event(ekST_ACQUIRE_BLOCK);
    return controller::foraging_signal::ekHANDLED;
  } else if (rpfsm::event_type::ekCHILD == data->type()) {
    if (controller::foraging_signal::ekENTERED_NEST == data->signal()) {
      internal_event(ekST_WAIT_FOR_DROP);
      return controller::foraging_signal::ekHANDLED;
    } else if (controller::foraging_signal::ekLEFT_NEST == data->signal()) {
      internal_event(ekST_ACQUIRE_BLOCK);
      return controller::foraging_signal::ekHANDLED;
    }
  }
  ER_FATAL_SENTINEL("Unhandled signal");
  return controller::foraging_signal::ekHANDLED;
}

HFSM_STATE_DEFINE_ND(cached_block_to_nest_fsm, acquire_block) {
  if (m_cache_fsm.task_finished()) {
    internal_event(ekST_WAIT_FOR_PICKUP);
  } else {
    m_cache_fsm.task_execute();
  }
  return controller::foraging_signal::ekHANDLED;
}

HFSM_STATE_DEFINE(cached_block_to_nest_fsm,
                  wait_for_pickup,
                  rpfsm::event_data* data) {
  if (controller::foraging_signal::ekBLOCK_PICKUP == data->signal()) {
    m_cache_fsm.task_reset();
    internal_event(ekST_TRANSPORT_TO_NEST);
  } else if (controller::foraging_signal::ekCACHE_VANISHED == data->signal()) {
    m_cache_fsm.task_reset();
    internal_event(ekST_ACQUIRE_BLOCK);
  }
  return controller::foraging_signal::ekHANDLED;
}

HFSM_STATE_DEFINE(cached_block_to_nest_fsm,
                  wait_for_drop,
                  rpfsm::event_data* data) {
  if (controller::foraging_signal::ekBLOCK_DROP == data->signal()) {
    m_cache_fsm.task_reset();
    internal_event(ekST_FINISHED);
  }
  return controller::foraging_signal::ekHANDLED;
}

__rcsw_const HFSM_STATE_DEFINE_ND(cached_block_to_nest_fsm, finished) {
  return controller::foraging_signal::ekHANDLED;
}

/*******************************************************************************
 * Collision Metrics
 ******************************************************************************/
__rcsw_pure bool cached_block_to_nest_fsm::in_collision_avoidance(void) const {
  return (m_cache_fsm.task_running() && m_cache_fsm.in_collision_avoidance()) ||
         base_foraging_fsm::in_collision_avoidance();
} /* in_collision_avoidance() */

__rcsw_pure bool cached_block_to_nest_fsm::entered_collision_avoidance(
    void) const {
  return (m_cache_fsm.task_running() &&
          m_cache_fsm.entered_collision_avoidance()) ||
         base_foraging_fsm::entered_collision_avoidance();
} /* entered_collision_avoidance() */

__rcsw_pure bool cached_block_to_nest_fsm::exited_collision_avoidance(void) const {
  return (m_cache_fsm.task_running() &&
          m_cache_fsm.exited_collision_avoidance()) ||
         base_foraging_fsm::exited_collision_avoidance();
} /* exited_collision_avoidance() */

__rcsw_pure rtypes::timestep cached_block_to_nest_fsm::collision_avoidance_duration(
    void) const {
  if (m_cache_fsm.task_running()) {
    return m_cache_fsm.collision_avoidance_duration();
  } else {
    return base_foraging_fsm::collision_avoidance_duration();
  }
} /* collision_avoidance_duration() */

__rcsw_pure rmath::vector2u cached_block_to_nest_fsm::avoidance_loc(void) const {
  if (m_cache_fsm.task_running()) {
    return m_cache_fsm.avoidance_loc();
  } else {
    return base_foraging_fsm::avoidance_loc();
  }
} /* collision_avoidance_duration() */

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
RCPPSW_WRAP_OVERRIDE_DEF(cached_block_to_nest_fsm,
                         is_exploring_for_goal,
                         m_cache_fsm,
                         const);

RCPPSW_WRAP_OVERRIDE_DEF(cached_block_to_nest_fsm,
                         is_vectoring_to_goal,
                         m_cache_fsm,
                         const);

RCPPSW_WRAP_OVERRIDE_DEF(cached_block_to_nest_fsm,
                         acquisition_loc,
                         m_cache_fsm,
                         const);

RCPPSW_WRAP_OVERRIDE_DEF(cached_block_to_nest_fsm,
                         current_explore_loc,
                         m_cache_fsm,
                         const);
RCPPSW_WRAP_OVERRIDE_DEF(cached_block_to_nest_fsm,
                         current_vector_loc,
                         m_cache_fsm,
                         const);

__rcsw_pure bool cached_block_to_nest_fsm::goal_acquired(void) const {
  if (acq_goal_type::ekEXISTING_CACHE == acquisition_goal()) {
    return current_state() == ekST_WAIT_FOR_PICKUP;
  } else if (transport_goal_type::ekNEST == block_transport_goal()) {
    return current_state() == ekST_WAIT_FOR_DROP;
  }
  return false;
}

__rcsw_pure acq_goal_type cached_block_to_nest_fsm::acquisition_goal(void) const {
  if (ekST_ACQUIRE_BLOCK == current_state() ||
      ekST_WAIT_FOR_PICKUP == current_state()) {
    return acq_goal_type::ekEXISTING_CACHE;
  } else if (ekST_ACQUIRE_BLOCK == current_state() ||
             ekST_WAIT_FOR_PICKUP == current_state()) {
    return acq_goal_type::ekEXISTING_CACHE;
  }
  return acq_goal_type::ekNONE;
} /* acquisition_goal() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
__rcsw_pure transport_goal_type
cached_block_to_nest_fsm::block_transport_goal(void) const {
  if (ekST_TRANSPORT_TO_NEST == current_state() ||
      ekST_WAIT_FOR_DROP == current_state()) {
    return transport_goal_type::ekNEST;
  }
  return transport_goal_type::ekNONE;
} /* block_transport_goal() */

void cached_block_to_nest_fsm::init(void) {
  base_foraging_fsm::init();
  m_cache_fsm.task_reset();
} /* init() */

void cached_block_to_nest_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::ekFSM_RUN,
               rpfsm::event_type::ekNORMAL);
} /* task_execute() */

NS_END(depth1, fsm, fordyca);
