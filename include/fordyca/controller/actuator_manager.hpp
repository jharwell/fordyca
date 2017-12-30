/**
 * @file actuator_manager.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_ACTUATOR_MANAGER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_ACTUATOR_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/core/utility/math/vector2.h>
#include "rcppsw/patterns/state_machine/simple_fsm.hpp"
#include "fordyca/params/actuator_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class actuator_manager
 *
 * @brief Handles the control of all actuators on the robot.
 *
 * Currently, that is:
 *
 * - \ref argos::CCI_DifferentSteeringActuator
 * - \ref argos::CCI_LEDsActuator
 * - \ref argos::CCI_RangeAndBearingActuator
 */
class actuator_manager: public state_machine::simple_fsm {
 public:
  actuator_manager(const struct params::actuator_params* c_params,
                   argos::CCI_DifferentialSteeringActuator* wheels,
                   argos::CCI_LEDsActuator* leds,
                   argos::CCI_RangeAndBearingActuator* raba);

  actuator_manager(const actuator_manager& fsm) = delete;
  actuator_manager& operator=(const actuator_manager& fsm) = delete;

  /**
   * @brief Set the color of the robot's LEDs.
   *
   * @param color The new color.
   */
  void leds_set_color(const argos::CColor& color) {
    m_leds->SetAllColors(color);
  }

  /*
   * @brief Gets a direction vector as input and transforms it into wheel
   * actuation commands
   *
   * @param heading The new heading. Note that the direction is relative
   * (i.e."change this much from the direction you are currently going in"), but
   * the magnitude is absolute (i.e. "change to this speed").
   *
   * @param force_hard_turn Whether or not a hard turn should be performed,
   * regardless of the angle difference. If this is not passed, then a hard turn
   * is performed only when the heading change is sufficiently different from
   * the current heading (as determined by paremeters).
   */
  void set_rel_heading(const argos::CVector2& heading,
                       bool force_hard_turn = false);
  void set_speed(double speed);

  /**
   * @brief Get the max wheel speed
   */
  double max_wheel_speed(void) const;

  /**
   * @brief Stop the robot
   */
  void stop_wheels(void) { m_wheels->SetLinearVelocity(0.0, 0.0); }
  void set_raba_data(uint8_t data) { m_raba->SetData(0, data); }

  /**
   * @brief Reset the actuators, including stopping the robot.
   */
  void reset(void);

  /**
   * @brief Direct control over the linear/angular speeds of the wheels.
   *
   * This provides an alternative interface much more precise rather than just
   * saying "go in this direction now" than you get with \ref
   * set_rel_heading(). However, it is also more difficult to use. Note that if
   * lin_speed + ang_speed is greater than the specified parameter value for max
   * wheel speed for either wheel it will saturate.
   *
   * @param lin_speed The desired linear speed.
   * @param ang_speed The desired angular speed.
   */
  void set_wheel_speeds(double lin_speed, double ang_speed);

 private:
  /**
   * @brief Set the wheel speeds according to the heading.
   *
   * @param speed1 Speed of one wheel.
   * @param speed2 Speed of the other wheel.
   * @param heading Robot heading, which is used to determine which speed to
   * apply to which wheel, so that the proper turn direction is executed.
   */
  void set_wheel_speeds(double speed1, double speed2, argos::CRadians heading);

  /*
   * @enum The robot can be in three different turning states.
   */
  enum fsm_states {
    ST_NO_TURN,     /// Go straight
    ST_SOFT_TURN,   /// Both wheels rotating forward at slightly different speeds
    ST_HARD_TURN,   /// Wheels are turning with opposite & max speeds
    ST_MAX_STATES
  };

  /**
   * @brief Turning data for input into the state machine, to translate the
   * desired heading change into wheel speeds.
   */
  struct turn_data : public state_machine::event_data {
    turn_data(argos::CVector2 heading_, bool force_hard_) :
        heading(heading_), force_hard(force_hard_) {}

    argos::CVector2 heading;
    bool force_hard;
  };

  /**
   * @brief Robots in this state will continue forward without turning.
   */
  FSM_STATE_DECLARE(actuator_manager, no_turn, turn_data);

  /**
   * @brief Robots in this state will execute a gradual turn in the desired
   * heading direction. Threshold for this type of turn is controlled by
   * parameters.
   */
  FSM_STATE_DECLARE(actuator_manager, soft_turn, turn_data);

  /**
   * @brief Robots in this state will execute an in-place turn (a spin really)
   * in the direction of the desired heading. Threshold for this type of turn
   * is controlled by parameters.
   */
  FSM_STATE_DECLARE(actuator_manager, hard_turn, turn_data);
  FSM_DEFINE_STATE_MAP_ACCESSOR(state_map, index) override {
    FSM_DEFINE_STATE_MAP(state_map, kSTATE_MAP) {
      FSM_STATE_MAP_ENTRY(&no_turn),
          FSM_STATE_MAP_ENTRY(&soft_turn),
          FSM_STATE_MAP_ENTRY(&hard_turn),
          };
    FSM_VERIFY_STATE_MAP(state_map, kSTATE_MAP, ST_MAX_STATES);
    return &kSTATE_MAP[index];
  }
  double m_lwheel_speed{0.0};
  double m_rwheel_speed{0.0};
  argos::CCI_DifferentialSteeringActuator* m_wheels;  /* differential steering */
  argos::CCI_LEDsActuator*                 m_leds;    /* LEDs  */
  argos::CCI_RangeAndBearingActuator*      m_raba;    /* Range and bearing */
  const struct params::actuator_params     mc_params;
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_ACTUATOR_MANAGER_HPP_ */
