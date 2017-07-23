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

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace fordyca {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class actuator_manager {
 public:
  /* constructors */
  actuator_manager(void);

  /* member functions */
  /*
   * Gets a direction vector as input and transforms it into wheel
   * actuation.
   */
  void SetWheelSpeeds(const argos::CVector2& c_heading);


 private:
  /* differential steering actuator */
  argos::CCI_DifferentialSteeringActuator* pc_wheels_;
  /* LEDs actuator */
  argos::CCI_LEDsActuator* pc_leds_;
  /* Range and bearing actuator */
  argos::CCI_RangeAndBearingActuator*  pr_raba_;
};

} /* namespace fordyca */

#endif /* INCLUDE_FORDYCA_ACTUATOR_MANAGER_HPP_ */
