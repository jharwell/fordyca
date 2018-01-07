/**
 * @file actuator_manager.cpp
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
#include "fordyca/controller/actuator_manager.hpp"
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include "fordyca/params/actuator_params.hpp"
#include "rcppsw/er/server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
actuator_manager::actuator_manager(
    const struct params::actuator_params *c_params,
    argos::CCI_DifferentialSteeringActuator *wheels,
    argos::CCI_LEDsActuator *leds,
    argos::CCI_RangeAndBearingActuator *raba)
    : state_machine::simple_fsm(rcppsw::er::g_server, ST_MAX_STATES),
      no_turn(),
      soft_turn(),
      hard_turn(),
      m_wheels(wheels),
      m_leds(leds),
      m_raba(raba),
      mc_params(*c_params) {}

/*******************************************************************************
 * Events
 ******************************************************************************/
void actuator_manager::set_rel_heading(const argos::CVector2 &heading,
                                   bool force_hard_turn) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
      ST_NO_TURN,   /* no turn */
      ST_SOFT_TURN, /* slow turn */
      ST_HARD_TURN, /* hard turn */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, ST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()],
                 rcppsw::make_unique<turn_data>(heading, force_hard_turn));
} /* set_rel_heading() */

/*******************************************************************************
 * States
 ******************************************************************************/
FSM_STATE_DEFINE(actuator_manager, no_turn, turn_data) {
  if (data->force_hard ||
      argos::Abs(data->heading.Angle().SignedNormalize()) >
          mc_params.wheels.soft_turn_max) {
    internal_event(ST_HARD_TURN);
  } else if (argos::Abs(data->heading.Angle().SignedNormalize()) >
             mc_params.wheels.no_turn_max) {
    internal_event(ST_SOFT_TURN);
  }
  set_wheel_speeds(data->heading.Length(),
                   data->heading.Length(),
                   data->heading.Angle());
  return state_machine::event_signal::HANDLED;
}
FSM_STATE_DEFINE(actuator_manager, soft_turn, turn_data) {
    if (argos::Abs(data->heading.Angle().SignedNormalize()) <=
            mc_params.wheels.no_turn_max) {
     internal_event(ST_NO_TURN);
    } else if (data->force_hard ||
               argos::Abs(data->heading.Angle().SignedNormalize()) >
               mc_params.wheels.soft_turn_max) {
      internal_event(ST_HARD_TURN);
    }

  /* Both wheels go straight, but one is faster than the other */
  double speed_factor =
      (mc_params.wheels.soft_turn_max - argos::Abs(data->heading.Angle())) /
      mc_params.wheels.soft_turn_max;
  double speed1 =
      data->heading.Length() - data->heading.Length() * (1.0 - speed_factor);
  double speed2 =
      data->heading.Length() + data->heading.Length() * (1.0 - speed_factor);
  set_wheel_speeds(speed1, speed2, data->heading.Angle());
  return state_machine::event_signal::HANDLED;
}
FSM_STATE_DEFINE(actuator_manager, hard_turn, turn_data) {
  if (argos::Abs(data->heading.Angle().SignedNormalize()) <=
                 mc_params.wheels.no_turn_max &&
             !data->force_hard) {
    internal_event(ST_NO_TURN);
  } else if (argos::Abs(data->heading.Angle().SignedNormalize()) <=
             mc_params.wheels.soft_turn_max &&
             !data->force_hard) {
    internal_event(ST_SOFT_TURN);
  }

  set_wheel_speeds(-mc_params.wheels.max_speed,
                   mc_params.wheels.max_speed,
                   data->heading.Angle());
  return state_machine::event_signal::HANDLED;
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
__pure double actuator_manager::clamp_wheel_speed(double desired) {
  double max_speed;
  if (m_throttle) {
    max_speed = mc_params.wheels.max_speed * (1.0 - m_throttle_percent);
  } else {
    max_speed = mc_params.wheels.max_speed;
  }
  return std::min(desired, max_speed);
} /* clamp_wheel_speed() */

void actuator_manager::set_speed(double speed) {
  m_lwheel_speed = clamp_wheel_speed(speed);
  m_rwheel_speed = clamp_wheel_speed(speed);
  m_wheels->SetLinearVelocity(m_lwheel_speed, m_rwheel_speed);
} /* set_speed() */

void actuator_manager::set_wheel_speeds(double speed1,
                                        double speed2,
                                        argos::CRadians heading) {
  if (heading > argos::CRadians::ZERO) {
    /* Turn Left */
    m_lwheel_speed = speed1;
    m_rwheel_speed = speed2;
  } else {
    /* Turn Right */
    m_lwheel_speed = speed2;
    m_rwheel_speed = speed1;
  }

  /* Finally, set the wheel speeds */
  m_lwheel_speed = clamp_wheel_speed(m_lwheel_speed);
  m_rwheel_speed = clamp_wheel_speed(m_rwheel_speed);
  m_wheels->SetLinearVelocity(m_lwheel_speed, m_rwheel_speed);
} /* set_wheel_speeds() */

void actuator_manager::set_wheel_speeds(double lin_speed, double ang_speed) {
  if (ang_speed < 0) {
    m_rwheel_speed = lin_speed + ang_speed;
    m_lwheel_speed = lin_speed;
  } else {
    m_lwheel_speed = lin_speed + ang_speed;
    m_rwheel_speed = lin_speed;
  }

  m_lwheel_speed = clamp_wheel_speed(m_lwheel_speed);
  m_rwheel_speed = clamp_wheel_speed(m_rwheel_speed);
  m_wheels->SetLinearVelocity(m_lwheel_speed, m_rwheel_speed);
} /* set_wheel_speeds() */

void actuator_manager::reset(void) {
  m_raba->ClearData();
  stop_wheels();
  state_machine::simple_fsm::init();
} /* reset() */

__pure double actuator_manager::max_wheel_speed(void) const {
  return mc_params.wheels.max_speed;
} /* max_wheel_speed() */

NS_END(controller, fordyca);
