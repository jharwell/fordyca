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
    return_to_nest(),
    leaving_nest(),
    collision_avoidance(),
    entry_explore(),
    entry_collision_avoidance(),
    entry_leaving_nest(),
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
void foraging_fsm::event_block_found(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS) {
    fsm::event_signal::FATAL,    /* start */
        ST_RETURN_TO_NEST,           /* explore */
        ST_RETURN_TO_NEST,           /* new direction */
        fsm::event_signal::IGNORED,  /* return to nest */
        fsm::event_signal::FATAL,    /* leaving nest */
        ST_COLLISION_AVOIDANCE,      /* collision avoidance */
        };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()], NULL);
}

void foraging_fsm::event_continue(void) {
  static const uint8_t kTRANSITIONS[] = {
    ST_EXPLORE,                  /* start */
    ST_EXPLORE,                 /* explore */
    ST_NEW_DIRECTION,           /* new direction */
    ST_RETURN_TO_NEST,          /* return to nest */
    ST_LEAVING_NEST,            /* leaving nest */
    ST_COLLISION_AVOIDANCE      /* collision avoidance */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()], NULL);
}

/*******************************************************************************
 * States
 ******************************************************************************/
FSM_STATE_DEFINE(foraging_fsm, leaving_nest, fsm::no_event_data) {
  ER_DIAG("Executing ST_LEAVING_NEST");

  /*
   * The vector returned by calc_vector_to_light() points to the light. Thus,
   * the minus sign is because we want to go away from the light.
   */
  argos::CVector2 diff_vector;
  argos::CRadians current_heading = m_sensors->calc_vector_to_light().Angle();
  m_sensors->calc_diffusion_vector(&diff_vector);
  m_actuators->set_heading(m_actuators->max_wheel_speed() * diff_vector -
                           argos::CVector2(m_actuators->max_wheel_speed() * 0.25f,
                                           current_heading));
  if (!m_sensors->in_nest()) {
    internal_event(ST_EXPLORE);
  }
  return fsm::event_signal::HANDLED;
}
FSM_STATE_DEFINE(foraging_fsm, explore, fsm::no_event_data) {
  ER_DIAG("Executing ST_EXPLORE");

  ++m_state.time_exploring_unsuccessfully;

  /*
   * Check for nearby obstacles, and if so go int obstacle avoidance. Time spent
   * in collision avoidance still counts towards the direction change threshold.
   */
  if (m_sensors->calc_diffusion_vector(NULL)) {
    internal_event(ST_COLLISION_AVOIDANCE);
  } else if (m_state.time_exploring_unsuccessfully >
             mc_params->times.unsuccessful_explore_dir_change) {
    argos::CRange<argos::CRadians> range(argos::CRadians(0.50),
                                         argos::CRadians(1.0));
    argos::CVector2 new_dir = argos::CVector2::X;
    new_dir = new_dir.Rotate(m_rng->Uniform(range));
    internal_event(ST_NEW_DIRECTION,
                   rcppsw::make_unique<new_direction_data>(new_dir.Angle()));
  }

  /*
   * No obstacles nearby and have not hit direction changing threshold--use the
   * diffusion vector only to set speeds.
   */
  argos::CVector2 vector;
  m_sensors->calc_diffusion_vector(&vector);
  m_actuators->set_heading(m_actuators->max_wheel_speed() * vector);
  return fsm::event_signal::HANDLED;
}
FSM_STATE_DEFINE(foraging_fsm, start, fsm::no_event_data) {
  /* argos::CRange<argos::CRadians> range(argos::CRadians(0.25), */
  /*                                      argos::CRadians(1.0)); */
  /* argos::CVector2 new_dir = argos::CVector2::X; */
  /* new_dir = new_dir.Rotate(m_rng->Uniform(range)); */
  /* printf("HERE\n"); */
  internal_event(ST_EXPLORE);
  return fsm::event_signal::HANDLED;
}
FSM_STATE_DEFINE(foraging_fsm, new_direction, new_direction_data) {
  ER_DIAG("Executing ST_NEW_DIRECTION");
  static argos::CRadians new_dir;
  static int count = 0;
  argos::CRadians current_dir = m_sensors->calc_vector_to_light().Angle();

  /*
   * The new direction is only passed the first time this state is entered, so
   * save it.
   */
  if (data) {
    count = 0;
    new_dir = data->dir;
  }
  m_actuators->set_heading(argos::CVector2(m_actuators->max_wheel_speed() * 0.25,
                                           new_dir), true);

  /* We have changed direction and started a new exploration */
  if (std::fabs((current_dir - new_dir).GetValue()) < 0.1) {
    m_state.time_exploring_unsuccessfully = 0;
    internal_event(ST_EXPLORE);
  }
  ++count;
  return fsm::event_signal::HANDLED;
}
FSM_STATE_DEFINE(foraging_fsm, return_to_nest, fsm::no_event_data) {
  ER_DIAG("Executing ST_RETURN_TO_NEST");
  argos::CVector2 vector;

  /*
   * We have arrived at the nest and it's time to head back out again. The
   * loop functions need to call the drop_block() function, as they have to
   * redistribute it (the FSM has no idea how to do that).
   */
  if (m_sensors->in_nest()) {
    internal_event(ST_LEAVING_NEST);
  }

  m_sensors->calc_diffusion_vector(&vector);
  m_actuators->set_heading(m_actuators->max_wheel_speed() * vector +
                           m_actuators->max_wheel_speed() * m_sensors->calc_vector_to_light());
  return fsm::event_signal::HANDLED;
}
FSM_STATE_DEFINE(foraging_fsm, collision_avoidance, fsm::no_event_data) {
  argos::CVector2 vector;
  ER_DIAG("Executing ST_COLLIISION_AVOIDANCE");
  if (m_sensors->calc_diffusion_vector(&vector)) {
    m_actuators->set_heading(vector);
  } else {
    internal_event(previous_state());
  }
  return fsm::event_signal::HANDLED;
}
FSM_ENTRY_DEFINE(foraging_fsm, entry_leaving_nest, fsm::no_event_data) {
  ER_DIAG("Entering ST_LEAVING_NEST");
  m_actuators->leds_set_color(argos::CColor::WHITE);
}
FSM_ENTRY_DEFINE(foraging_fsm, entry_explore, fsm::no_event_data) {
  ER_DIAG("Entering ST_EXPLORE");
  m_actuators->leds_set_color(argos::CColor::MAGENTA);
}
FSM_ENTRY_DEFINE(foraging_fsm, entry_new_direction, fsm::no_event_data) {
  ER_DIAG("Entering ST_NEW_DIRECTION");
  m_actuators->leds_set_color(argos::CColor::CYAN);
}
FSM_ENTRY_DEFINE(foraging_fsm, entry_return_to_nest, fsm::no_event_data) {
  ER_DIAG("Entering ST_RETURN_TO_NEST");
  m_actuators->leds_set_color(argos::CColor::GREEN);
}
FSM_ENTRY_DEFINE(foraging_fsm, entry_collision_avoidance, fsm::no_event_data) {
  ER_DIAG("Entering ST_COLLIISION_AVOIDANCE");
  m_actuators->leds_set_color(argos::CColor::RED);
}

FSM_EXIT_DEFINE(foraging_fsm, exit_leaving_nest) {
  ER_DIAG("Exiting ST_LEAVING_NEST");
  m_state.time_exploring_unsuccessfully = 0;
}
/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void foraging_fsm::init(void) {
  m_state.time_exploring_unsuccessfully = 0;
  m_actuators->reset();
  simple_fsm::init();
} /* init() */

NS_END(controller, fordyca);
