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
namespace rfsm = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
cached_block_to_nest_fsm::cached_block_to_nest_fsm(
    const controller::cache_sel_matrix* sel_matrix,
    controller::saa_subsystem* const saa,
    ds::dpo_store* const store)
    : base_foraging_fsm(saa, ST_MAX_STATES),
      ER_CLIENT_INIT("fordyca.fsm.depth1.cached_block_to_nest"),
      HFSM_CONSTRUCT_STATE(transport_to_nest, &start),
      HFSM_CONSTRUCT_STATE(leaving_nest, &start),
      entry_transport_to_nest(),
      entry_leaving_nest(),
      entry_wait_for_signal(),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_block, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_pickup, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_drop, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      m_cache_fsm(sel_matrix, true, saa, store),
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

HFSM_STATE_DEFINE(cached_block_to_nest_fsm, start, rfsm::event_data* data) {
  if (rfsm::event_type::NORMAL == data->type()) {
    internal_event(ST_ACQUIRE_BLOCK);
    return controller::foraging_signal::HANDLED;
  } else if (rfsm::event_type::CHILD == data->type()) {
    if (controller::foraging_signal::ENTERED_NEST == data->signal()) {
      internal_event(ST_WAIT_FOR_DROP);
      return controller::foraging_signal::HANDLED;
    } else if (controller::foraging_signal::LEFT_NEST == data->signal()) {
      internal_event(ST_ACQUIRE_BLOCK);
      return controller::foraging_signal::HANDLED;
    }
  }
  ER_FATAL_SENTINEL("Unhandled signal");
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE_ND(cached_block_to_nest_fsm, acquire_block) {
  if (m_cache_fsm.task_finished()) {
    internal_event(ST_WAIT_FOR_PICKUP);
  } else {
    m_cache_fsm.task_execute();
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE(cached_block_to_nest_fsm,
                  wait_for_pickup,
                  rfsm::event_data *data) {
  if (controller::foraging_signal::BLOCK_PICKUP == data->signal()) {
    m_cache_fsm.task_reset();
    internal_event(ST_TRANSPORT_TO_NEST);
  } else if (controller::foraging_signal::CACHE_VANISHED == data->signal()) {
    m_cache_fsm.task_reset();
    internal_event(ST_ACQUIRE_BLOCK);
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE(cached_block_to_nest_fsm,
                  wait_for_drop,
                  rfsm::event_data *data) {
  if (controller::foraging_signal::BLOCK_DROP == data->signal()) {
    m_cache_fsm.task_reset();
    internal_event(ST_FINISHED);
  }
  return controller::foraging_signal::HANDLED;
}

__rcsw_const HFSM_STATE_DEFINE_ND(cached_block_to_nest_fsm, finished) {
  return controller::foraging_signal::HANDLED;
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

__rcsw_pure uint
cached_block_to_nest_fsm::collision_avoidance_duration(void) const {
  if (m_cache_fsm.task_running()) {
    return m_cache_fsm.collision_avoidance_duration();
  } else {
    return base_foraging_fsm::collision_avoidance_duration();
  }
} /* collision_avoidance_duration() */

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
FSM_OVERRIDE_DEF(bool,
                 cached_block_to_nest_fsm,
                 is_exploring_for_goal,
                 m_cache_fsm,
                 const);

FSM_OVERRIDE_DEF(bool,
                 cached_block_to_nest_fsm,
                 is_vectoring_to_goal,
                 m_cache_fsm,
                 const);

bool cached_block_to_nest_fsm::goal_acquired(void) const {
  if (acquisition_goal_type::kExistingCache == acquisition_goal()) {
    return current_state() == ST_WAIT_FOR_PICKUP;
  } else if (transport_goal_type::kNest == block_transport_goal()) {
    return current_state() == ST_WAIT_FOR_DROP;
  }
  return false;
}

acquisition_goal_type cached_block_to_nest_fsm::acquisition_goal(void) const {
  if (ST_ACQUIRE_BLOCK == current_state() ||
      ST_WAIT_FOR_PICKUP == current_state()) {
    return acquisition_goal_type::kExistingCache;
  } else if (ST_ACQUIRE_BLOCK == current_state() ||
             ST_WAIT_FOR_PICKUP == current_state()) {
    return acquisition_goal_type::kExistingCache;
  }
  return acquisition_goal_type::kNone;
} /* acquisition_goal() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
transport_goal_type cached_block_to_nest_fsm::block_transport_goal(void) const {
  if (ST_TRANSPORT_TO_NEST == current_state() ||
      ST_WAIT_FOR_DROP == current_state()) {
    return transport_goal_type::kNest;
  }
  return transport_goal_type::kNone;
} /* block_transport_goal() */

void cached_block_to_nest_fsm::init(void) {
  base_foraging_fsm::init();
  m_cache_fsm.task_reset();
} /* init() */

void cached_block_to_nest_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               rfsm::event_type::NORMAL);
} /* task_execute() */

NS_END(depth1, fsm, fordyca);
