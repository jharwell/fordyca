/**
 * @file random_foraging_fsm.cpp
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
#include "fordyca/controller/random_foraging_fsm.hpp"
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
namespace fsm = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
random_foraging_fsm::random_foraging_fsm(
    const struct foraging_fsm_params* params,
    std::shared_ptr<rcppsw::common::er_server> server,
    std::shared_ptr<sensor_manager> sensors,
    std::shared_ptr<actuator_manager> actuators) :
    fsm::hfsm(server),
    HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(explore, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(new_direction, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(return_to_nest, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(leaving_nest, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(collision_avoidance, hfsm::top_state()),
    entry_explore(),
    entry_new_direction(),
    entry_return_to_nest(),
    entry_collision_avoidance(),
    entry_leaving_nest(),
    exit_leaving_nest(),
    m_current_state(ST_START),
    m_next_state(ST_START),
    m_initial_state(ST_START),
    m_previous_state(ST_START),
    m_last_state(ST_START),
    m_rng(argos::CRandom::CreateRNG("argos")),
    m_state(),
    mc_params(params),
    m_sensors(sensors),
    m_actuators(actuators) {
  insmod("random_foraging_fsm");
  server_handle()->mod_loglvl(er_id(), rcppsw::common::er_lvl::DIAG);
  server_handle()->mod_dbglvl(er_id(), rcppsw::common::er_lvl::NOM);
}

/*******************************************************************************
 * Events
 ******************************************************************************/
void random_foraging_fsm::event_block_found(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS) {
        ST_RETURN_TO_NEST,           /* start (robot might be on block initially) */
        ST_RETURN_TO_NEST,           /* explore */
        ST_RETURN_TO_NEST,           /* new direction */
        fsm::event_signal::IGNORED,  /* return to nest */
        fsm::event_signal::FATAL,    /* leaving nest */
        ST_COLLISION_AVOIDANCE,      /* collision avoidance */
        };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()], NULL);
}

void random_foraging_fsm::event_start(void) {
  static const uint8_t kTRANSITIONS[] = {
    ST_EXPLORE,                  /* start */
    ST_EXPLORE,                  /* explore */
    fsm::event_signal::IGNORED,  /* new direction */
    fsm::event_signal::IGNORED,  /* return to nest */
    fsm::event_signal::IGNORED,  /* leaving nest */
    fsm::event_signal::IGNORED,  /* collision avoidance */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()], NULL);
}

/*******************************************************************************
 * States
 ******************************************************************************/
FSM_STATE_DEFINE(random_foraging_fsm, leaving_nest, fsm::no_event_data) {
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
FSM_STATE_DEFINE(random_foraging_fsm, explore, fsm::no_event_data) {
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
    argos::CVector2 new_dir = randomize_vector_angle(argos::CVector2::X);
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
FSM_STATE_DEFINE(random_foraging_fsm, start, fsm::no_event_data) {
  internal_event(ST_EXPLORE);
  return fsm::event_signal::HANDLED;
}
FSM_STATE_DEFINE(random_foraging_fsm, new_direction, new_direction_data) {
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
FSM_STATE_DEFINE(random_foraging_fsm, return_to_nest, fsm::no_event_data) {
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


  /* ignore all obstacles for now... */
  m_sensors->calc_diffusion_vector(&vector);
  m_actuators->set_heading(m_actuators->max_wheel_speed() * vector +
                           m_actuators->max_wheel_speed() * m_sensors->calc_vector_to_light());
  return fsm::event_signal::HANDLED;
}
FSM_STATE_DEFINE(random_foraging_fsm, collision_avoidance, fsm::no_event_data) {
  argos::CVector2 vector;
  ER_DIAG("Executing ST_COLLIISION_AVOIDANCE");

  if (m_sensors->calc_diffusion_vector(&vector)) {
    m_actuators->set_heading(vector);
  } else {
    internal_event(previous_state());
  }
  return fsm::event_signal::HANDLED;
}
FSM_ENTRY_DEFINE(random_foraging_fsm, entry_leaving_nest, fsm::no_event_data) {
  ER_DIAG("Entering ST_LEAVING_NEST");
  m_actuators->leds_set_color(argos::CColor::WHITE);
}
FSM_ENTRY_DEFINE(random_foraging_fsm, entry_explore, fsm::no_event_data) {
  ER_DIAG("Entering ST_EXPLORE");
  m_actuators->leds_set_color(argos::CColor::MAGENTA);
}
FSM_ENTRY_DEFINE(random_foraging_fsm, entry_new_direction, fsm::no_event_data) {
  ER_DIAG("Entering ST_NEW_DIRECTION");
  m_actuators->leds_set_color(argos::CColor::CYAN);
}
FSM_ENTRY_DEFINE(random_foraging_fsm, entry_return_to_nest, fsm::no_event_data) {
  ER_DIAG("Entering ST_RETURN_TO_NEST");
  m_actuators->leds_set_color(argos::CColor::GREEN);
}
FSM_ENTRY_DEFINE(random_foraging_fsm, entry_collision_avoidance, fsm::no_event_data) {
  ER_DIAG("Entering ST_COLLISION_AVOIDANCE");
  m_actuators->leds_set_color(argos::CColor::RED);
  m_state.last_collision_time = m_sensors->tick();
}

FSM_EXIT_DEFINE(random_foraging_fsm, exit_leaving_nest) {
  ER_DIAG("Exiting ST_LEAVING_NEST");
  m_state.time_exploring_unsuccessfully = 0;
}
/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void random_foraging_fsm::init(void) {
  m_state.time_exploring_unsuccessfully = 0;
  m_actuators->reset();
  hfsm::init();
} /* init() */

argos::CVector2 random_foraging_fsm::randomize_vector_angle(argos::CVector2 vector) {
  argos::CRange<argos::CRadians> range(argos::CRadians(0.0),
                                       argos::CRadians(1.0));
  vector.Rotate(m_rng->Uniform(range));
  return vector;
} /* randomize_vector_angle() */

bool random_foraging_fsm::is_searching_for_block(void) {
  return current_state() == ST_EXPLORE || current_state() == ST_NEW_DIRECTION;
} /* is_searching_for_block() */

bool random_foraging_fsm::is_returning(void) {
  return current_state() == ST_RETURN_TO_NEST;
} /* is_returning() */

bool random_foraging_fsm::is_avoiding_collision(void) {
  return current_state() == ST_COLLISION_AVOIDANCE;
} /* is_avoiding_collision() */

void random_foraging_fsm::update_state(uint8_t new_state) {
  if (new_state != m_current_state) {
    m_previous_state = m_current_state;
  }
  m_last_state = m_current_state;
  m_current_state = new_state;
} /* update_state() */

NS_END(controller, fordyca);
