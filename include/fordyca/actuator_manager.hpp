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

#ifndef INCLUDE_FORDYCA_ACTUATOR_MANAGER_HPP_
#define INCLUDE_FORDYCA_ACTUATOR_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/core/utility/math/vector2.h>
#include "fordyca/fordyca_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class actuator_manager {
 public:
  /* constructors */
  actuator_manager(const struct actuator_params* params,
                   argos::CCI_DifferentialSteeringActuator* const wheels,
                   argos::CCI_LEDsActuator* const leds,
                   argos::CCI_RangeAndBearingActuator* const raba);

  /* member functions */
  void leds_set_color(const argos::CColor& color) {
    m_leds->SetAllColors(color);
  }

  /*
   * Gets a direction vector as input and transforms it into wheel
   * actuation.
   */
  void set_wheel_speeds(const argos::CVector2& c_heading);
  size_t max_wheel_speed(void) { return mc_params->wheels.max_speed; }
  void stop_wheels(void) { m_wheels->SetLinearVelocity(0.0f, 0.0f); }
  void set_raba_data(int data) { m_raba->SetData(0, data); }
  void reset(int state);

 private:
  actuator_manager(const actuator_manager& fsm) = delete;
  actuator_manager& operator=(const actuator_manager& fsm) = delete;

  /*
   * The robot can be in three different turning states.
   */
  enum turning_state {
    NO_TURN = 0, // go straight
    SOFT_TURN,   // both wheels are turning forwards, but at different speeds
    HARD_TURN    // wheels are turning with opposite speeds
  };

  enum turning_state                       m_turning_state;
  std::shared_ptr<argos::CCI_DifferentialSteeringActuator> m_wheels;  /* differential steering */
  std::shared_ptr<argos::CCI_LEDsActuator>                 m_leds;    /* LEDs  */
  std::shared_ptr<argos::CCI_RangeAndBearingActuator>      m_raba;    /* Range and bearing */
  std::shared_ptr<const struct actuator_params>  mc_params;

};

NS_END(fordyca);

#endif /* INCLUDE_FORDYCA_ACTUATOR_MANAGER_HPP_ */
