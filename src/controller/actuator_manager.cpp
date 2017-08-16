/**
 * @file actuator_manager.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
 *
 * This file is part of RCPPSW.
 *
 * RCPPSW is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * RCPPSW is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * RCPPSW.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/actuator_manager.hpp"
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
actuator_manager::actuator_manager(
    const struct actuator_params* params,
    argos::CCI_DifferentialSteeringActuator* const wheels,
    argos::CCI_LEDsActuator* const leds,
    argos::CCI_RangeAndBearingActuator* const raba) :
    simple_fsm(rcppsw::common::g_null_server, ST_MAX_STATES),
    no_turn(),
    soft_turn(),
    hard_turn(),
    m_wheels(wheels),
    m_leds(leds),
    m_raba(raba),
    mc_params(params) {}

/*******************************************************************************
 * Events
 ******************************************************************************/
void actuator_manager::set_heading(const argos::CVector2& heading,
                                   bool force_hard_turn) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS) {
        ST_NO_TURN,    /* no turn */
        ST_SOFT_TURN,  /* slow turn */
        ST_HARD_TURN,  /* hard turn */
        };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()],
                 rcppsw::make_unique<turn_data>(heading, force_hard_turn));
} /* set_heading() */

/*******************************************************************************
 * States
 ******************************************************************************/
FSM_STATE_DEFINE(actuator_manager, no_turn, turn_data) {
  if (data->force_hard ||
      Abs(data->heading.Angle().SignedNormalize()) >
      mc_params->wheels.hard_turn_threshold) {
    internal_event(ST_HARD_TURN);
  } else if (Abs(data->heading.Angle().SignedNormalize()) >
             mc_params->wheels.soft_turn_threshold) {
    internal_event(ST_SOFT_TURN);
  }
  set_wheel_speeds(data->heading.Length(), data->heading.Length(),
                   data->heading.Angle());
  return fsm::event_signal::HANDLED;
}
FSM_STATE_DEFINE(actuator_manager, soft_turn, turn_data) {
  if (data->force_hard ||
      Abs(data->heading.Angle().SignedNormalize()) >
      mc_params->wheels.hard_turn_threshold) {
    internal_event(ST_HARD_TURN);
  } else if (Abs(data->heading.Angle().SignedNormalize()) <=
             mc_params->wheels.no_turn_threshold) {
    internal_event(ST_NO_TURN);
  }
  /* Both wheels go straight, but one is faster than the other */
  argos::Real speed_factor = (mc_params->wheels.hard_turn_threshold -
                              Abs(data->heading.Angle())) /
                             mc_params->wheels.hard_turn_threshold;
  double speed1 = data->heading.Length() - data->heading.Length() *
                  (1.0 - speed_factor);
  double speed2 = data->heading.Length() + data->heading.Length() *
                  (1.0 - speed_factor);
  set_wheel_speeds(speed1, speed2, data->heading.Angle());
  return fsm::event_signal::HANDLED;
}
FSM_STATE_DEFINE(actuator_manager, hard_turn, turn_data) {
  if (Abs(data->heading.Angle().SignedNormalize()) <
      mc_params->wheels.hard_turn_threshold && !data->force_hard) {
    internal_event(ST_SOFT_TURN);
  } else if (Abs(data->heading.Angle().SignedNormalize()) <=
             mc_params->wheels.no_turn_threshold && !data->force_hard) {
    internal_event(ST_NO_TURN);
  }
  set_wheel_speeds(-mc_params->wheels.max_speed, mc_params->wheels.max_speed,
                   data->heading.Angle());
  return fsm::event_signal::HANDLED;
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void actuator_manager::set_wheel_speeds(double speed1, double speed2,
                                        argos::CRadians heading) {
  argos::Real left_wheel_speed, right_wheel_speed;
  if (heading > argos::CRadians::ZERO) {
    /* Turn Left */
    left_wheel_speed  = speed1;
    right_wheel_speed = speed2;
  } else {
    /* Turn Right */
    left_wheel_speed  = speed2;
    right_wheel_speed = speed1;
  }

  /* Finally, set the wheel speeds */
  left_wheel_speed = argos::Min<argos::Real>(left_wheel_speed,
                                             mc_params->wheels.max_speed);
  right_wheel_speed = argos::Min<argos::Real>(right_wheel_speed,
                                             mc_params->wheels.max_speed);
  m_wheels->SetLinearVelocity(left_wheel_speed, right_wheel_speed);
} /* set_wheel_speeds() */

void actuator_manager::reset(void) {
  m_raba->ClearData();
  simple_fsm::init();
} /* reset() */

NS_END(controller, fordyca);