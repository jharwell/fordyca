/**
 * @file ee_max_fsm.cpp
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
#include "fordyca/fsm/depth0/ee_max_fsm.hpp"
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/controller/random_explore_behavior.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth0);
namespace state_machine = rcppsw::patterns::state_machine;
namespace ta = rcppsw:task_allocation;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

// get pointer to taskable object (crw_fsm)
ee_max_fsm::ee_max_fsm(ta::taskable* const task, const controller::ee_decision_matrix* matrix)
    : base_foraging_fsm(saa, ST_MAX_STATES),
      ta::taskable* taskable_fsm(task),
      mc_matrix(matrix),
      ER_CLIENT_INIT("fordyca.fsm.depth0.ee_max"),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(foraging, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(retreating, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(charging, hfsm::top_state()),
      mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
                   HFSM_STATE_MAP_ENTRY_EX(&foraging),
                   HFSM_STATE_MAP_ENTRY_EX(&retreating),
                   HFSM_STATE_MAP_ENTRY_EX(&charging)} {}

/*******************************************************************************
 * States
 ******************************************************************************/
HFSM_STATE_DEFINE(ee_max_fsm, start, state_machine::event_data) {
  /* first time running FSM */

}

HFSM_STATE_DEFINE_ND(ee_max_fsm, foraging) {
  controller::ee_selector selector(mc_matrix);
  float low_energy = selector.getLowerThres();
  if (current_energy <= low_energy) {
    internal_event(ST_RETREATING);
    return;
  }

  if(block picked up) {
    internal_event(ST_RETREATING);
  }

}

HFSM_STATE_DEFINE_ND(ee_max_fsm, retreating) {
  if(robot is in nest) {
    // update threshold values?
    internal_event(ST_CHARGING);
  }

}

HFSM_STATE_DEFINE_ND(ee_max_fsm, charging) {
  controller::ee_selector selector(mc_matrix);
  float charged_energy = selector.getHigherThres();
  if (current_energy == charged_energy) {
    internal_event(ST_FORAGING);
    return;
  }
}

/*
HFSM_STATE_DEFINE_ND(ee_max_fsm, acquire_block) {
  if (m_explore_fsm.task_finished()) {
    internal_event(ST_WAIT_FOR_BLOCK_PICKUP);
  } else {
    m_explore_fsm.task_execute();
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE(ee_max_fsm, wait_for_block_pickup, state_machine::event_data) {
  if (controller::foraging_signal::BLOCK_PICKUP == data->signal()) {
    m_explore_fsm.task_reset();
    ER_INFO("Block pickup signal received");
    internal_event(ST_RETURN_TO_NEST);
    // Tony: robot will also enter this state when battery is low handled individually by robot
  } else if (controller::foraging_signal::BLOCK_VANISHED == data->signal()) {
    m_explore_fsm.task_reset();
    internal_event(ST_ACQUIRE_BLOCK);
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE(ee_max_fsm, wait_for_block_drop, state_machine::event_data) {
  if (controller::foraging_signal::BLOCK_DROP == data->signal()) {
    m_explore_fsm.task_reset();
    ER_INFO("Block drop signal received");
    internal_event(ST_LEAVING_NEST);
  }
  return controller::foraging_signal::HANDLED;

  FSM_STATE_DEFINE_ND(ee_max_fsm, charging) {
    if (m_explore_fsm.task_finished()) {
      internal_event(ST_CHARGING);
    }
    // Tony: robot will handle next when to exit nest aka charged, time spent, and P-val allowed
  }
}
*/

/*******************************************************************************
 * Metrics
 ******************************************************************************/
/*
bool ee_max_fsm::is_exploring_for_goal(void) const {
  return current_state() == ST_ACQUIRE_BLOCK;
} /* is_exploring_for_goal()

bool ee_max_fsm::goal_acquired(void) const {
  if (acquisition_goal_type::kBlock == acquisition_goal()) {
    return current_state() == ST_WAIT_FOR_BLOCK_PICKUP;
  } else if (transport_goal_type::kNest == block_transport_goal()) {
    return current_state() == ST_WAIT_FOR_BLOCK_DROP;
  }
  return false;
} /* goal_acquired() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/

/*
void ee_max_fsm::init(void) {
  base_foraging_fsm::init();
  m_explore_fsm.init();
} /* init()

void ee_max_fsm::run(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* run()

bool ee_max_fsm::block_detected(void) const {
  return saa_subsystem()->sensing()->block_detected();
} /* block_detected()

transport_goal_type ee_max_fsm::block_transport_goal(void) const {
  if (ST_RETURN_TO_NEST == current_state() ||
      ST_WAIT_FOR_BLOCK_DROP == current_state()) {
    return transport_goal_type::kNest;
  }
  return transport_goal_type::kNone;
} /* block_transport_goal()

acquisition_goal_type ee_max_fsm::acquisition_goal(void) const {
  if (ST_ACQUIRE_BLOCK == current_state() ||
      ST_WAIT_FOR_BLOCK_PICKUP == current_state()) {
    return acquisition_goal_type::kBlock;
  }
  return acquisition_goal_type::kNone;
} /* block_transport_goal() */

NS_END(depth0, fsm, fordyca);
