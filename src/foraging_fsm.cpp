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
#include "fordyca/controller/foraging_fsm.hpp"
#include <argos3/core/utility/datatypes/color.h>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
namespace fsm = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
foraging_fsm::foraging_fsm(const struct foraging_fsm_params* params,
                       std::shared_ptr<rcppsw::common::er_server> server,
                       std::shared_ptr<sensor_manager> sensors,
                       std::shared_ptr<actuator_manager> actuators) :
    fsm::simple_fsm(server, ST_MAX_STATES),
    explore(),
    explore_success(),
    explore_fail(),
    return_to_nest(),
    leaving_nest(),
    collision_avoidance(),
    entry_explore(),
    entry_collision_avoidance(),
    entry_leaving_nest(),
    m_last_explore_res(LAST_EXPLORATION_NONE),
    m_rng(argos::CRandom::CreateRNG("argos")),
    m_prob_range(0.0f, 1.0f),
    m_state(),
    mc_params(params),
    m_sensors(sensors),
    m_actuators(actuators) {
  insmod("foraging_fsm");
  server_handle()->mod_loglvl(er_id(), rcppsw::common::er_lvl::DIAG);
  server_handle()->mod_dbglvl(er_id(), rcppsw::common::er_lvl::NOM);
    }

/*******************************************************************************
 * Events
 ******************************************************************************/
void foraging_fsm::event_explore(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS) {
    fsm::event::EVENT_IGNORED,  /* explore */
    fsm::event::EVENT_IGNORED,  /* explore success */
    fsm::event::EVENT_IGNORED,  /* explore fail */
    fsm::event::EVENT_IGNORED,  /* return to nest */
    fsm::event::EVENT_IGNORED,  /* leaving nest */
    ST_COLLISION_AVOIDANCE,     /* collision avoidance */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()], NULL);
}

void foraging_fsm::event_block_found(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS) {
    ST_EXPLORE_SUCCESS,          /* explore */
    fsm::event::EVENT_IGNORED,   /* explore success */
    fsm::event::EVENT_IGNORED,   /* explore fail */
    fsm::event::EVENT_IGNORED,   /* return to nest */
    fsm::event::EVENT_FATAL,     /* leaving to nest */
    ST_COLLISION_AVOIDANCE,      /* collision avoidance */
        };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()], NULL);
}

void foraging_fsm::event_continue(void) {
  static const uint8_t kTRANSITIONS[] = {
    ST_EXPLORE,                 /* explore */
    ST_EXPLORE_SUCCESS,         /* explore success */
    ST_EXPLORE_FAIL,            /* explore fail */
    ST_RETURN_TO_NEST,          /* return to nest */
    ST_LEAVING_NEST,            /* leaving to nest */
    ST_COLLISION_AVOIDANCE      /* collision avoidance */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()], NULL);
}

/*******************************************************************************
 * States
 ******************************************************************************/
FSM_STATE_DEFINE(foraging_fsm, leaving_nest, fsm::no_event) {
  ER_DIAG("Executing ST_LEAVING_NEST");

  /*
   * The vector returned by calc_vector_to_light() points to
   * the light. Thus, the minus sign is because we want to go away
   * from the light.
   */
  argos::CVector2 diff_vector;
  m_sensors->calc_diffusion_vector(&diff_vector);
  m_actuators->set_wheel_speeds(
      m_actuators->max_wheel_speed() * diff_vector -
      m_actuators->max_wheel_speed() * 0.25f * m_sensors->calc_vector_to_light());
  if (!m_sensors->in_nest()) {
    internal_event(ST_EXPLORE);
  }
  return fsm::event::EVENT_HANDLED;
}
FSM_STATE_DEFINE(foraging_fsm, explore, fsm::no_event) {
  ER_DIAG("Executing ST_EXPLORE");

  /*
   * We transition to the 'return to nest' state in two situations:
   *
   * 1. If we have a block item, in which case we are notified by an external event.
   * 2. If we have not found a block item for some time; in this case, the switch
   *    is probabilistic.
   */

  /*
   * Second condition: we probabilistically switch to 'return to
   * nest' if we have been wandering for some time and found nothing.
   */
  if (m_state.time_exploring_unsuccessfully >
      mc_params->times.max_unsuccessful_explore &&
    m_rng->Uniform(m_prob_range) < m_state.explore_to_rest_prob) {
      internal_event(ST_EXPLORE_FAIL);
  }

  /* perform the actual exploration */
  ++m_state.time_exploring_unsuccessfully;

  /* Get the diffusion vector to perform obstacle avoidance */
  if (m_sensors->calc_diffusion_vector(NULL)) {
    internal_event(ST_COLLISION_AVOIDANCE);
  }
  argos::CVector2 diff_vector;
  m_sensors->calc_diffusion_vector(&diff_vector);

  /* No obstacles nearby--use the diffusion vector only to set speeds */
  m_actuators->set_wheel_speeds(m_actuators->max_wheel_speed() * diff_vector);
  return fsm::event::EVENT_HANDLED;
}
FSM_STATE_DEFINE(foraging_fsm, explore_success, fsm::no_event) {
  ER_DIAG("Executing ST_EXPLORE_SUCCESS");

  /* Store the result of the expedition */
  m_last_explore_res = LAST_EXPLORATION_SUCCESSFUL;
  internal_event(ST_RETURN_TO_NEST);
  return fsm::event::EVENT_HANDLED;
}
FSM_STATE_DEFINE(foraging_fsm, explore_fail, fsm::no_event) {
  ER_DIAG("Executing ST_EXPLORE_FAIL");

  /* Store the result of the expedition */
  m_last_explore_res = LAST_EXPLORATION_UNSUCCESSFUL;
  internal_event(ST_RETURN_TO_NEST);
  return fsm::event::EVENT_HANDLED;
}
FSM_STATE_DEFINE(foraging_fsm, return_to_nest, fsm::no_event) {
    ER_DIAG("Executing ST_RETURN_TO_NEST");
    argos::CVector2 vector;

    if (LAST_EXPLORATION_SUCCESSFUL == m_last_explore_res) {
      m_actuators->leds_set_color(argos::CColor::GREEN);
    } else {
      m_actuators->leds_set_color(argos::CColor::ORANGE);
    }

    if (m_sensors->in_nest()) {
      internal_event(ST_LEAVING_NEST);
    }

    m_sensors->calc_diffusion_vector(&vector);
    m_actuators->set_wheel_speeds(
        m_actuators->max_wheel_speed() * vector +
        m_actuators->max_wheel_speed() * m_sensors->calc_vector_to_light());
      return fsm::event::EVENT_HANDLED;
}

FSM_STATE_DEFINE(foraging_fsm, collision_avoidance, fsm::no_event) {
  argos::CVector2 vector;
  static argos::CColor last_color;
  ER_DIAG("Executing ST_COLLIISION_AVOIDANCE");
  if (m_sensors->calc_diffusion_vector(&vector)) {
    m_actuators->set_wheel_speeds(vector);
  } else {
    internal_event(previous_state());
  }
  return fsm::event::EVENT_HANDLED;
}
FSM_ENTRY_DEFINE(foraging_fsm, entry_leaving_nest, fsm::no_event) {
  ER_DIAG("Entering ST_LEAVING_NEST");
  m_actuators->leds_set_color(argos::CColor::WHITE);
}
FSM_ENTRY_DEFINE(foraging_fsm, entry_explore, fsm::no_event) {
  ER_DIAG("Entrying ST_EXPLORE");
  m_actuators->leds_set_color(argos::CColor::MAGENTA);
  m_state.time_exploring_unsuccessfully = 0;
  m_last_explore_res = LAST_EXPLORATION_NONE;
}
FSM_ENTRY_DEFINE(foraging_fsm, entry_collision_avoidance, fsm::no_event) {
  ER_DIAG("Entering ST_COLLIISION_AVOIDANCE");
  m_actuators->leds_set_color(argos::CColor::RED);
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void foraging_fsm::init(void) {
  m_state.time_exploring_unsuccessfully = 0;
  m_last_explore_res = LAST_EXPLORATION_NONE;
  m_actuators->reset(LAST_EXPLORATION_NONE);
  simple_fsm::init();
} /* init() */

NS_END(controller, fordyca);
