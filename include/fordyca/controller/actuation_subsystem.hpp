/**
 * @file actuation_subsystem.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_ACTUATION_SUBSYSTEM_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_ACTUATION_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/steering_force2D.hpp"
#include "fordyca/controller/throttling_differential_drive.hpp"
#include "fordyca/params/actuation_params.hpp"
#include "rcppsw/robotics/hal/actuators/differential_drive_actuator.hpp"
#include "rcppsw/robotics/hal/actuators/led_actuator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace argos {
class CCI_RangeAndBearingActuator;
} // namespace argos

NS_START(fordyca, controller);

namespace state_machine = rcppsw::patterns::state_machine;
namespace hal = rcppsw::robotics::hal;
namespace utils = rcppsw::utils;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class actuation_subsystem
 * @ingroup controller
 *
 * @brief Handles the control of all actuators on the robot.
 *
 * Currently, that is:
 *
 * - argos::CCI_DifferentSteeringActuator
 * - argos::CCI_LEDsActuator
 * - argos::CCI_RangeAndBearingActuator
 */
class actuation_subsystem {
 public:
  struct actuator_list {
    hal::actuators::differential_drive_actuator wheels;
    hal::actuators::led_actuator leds;
    argos::CCI_RangeAndBearingActuator* raba;
  };

  /**
   * @brief Initialize the actuation subsystem.
   *
   * @param c_params Subsystem parameters.
   * @param list List of handles to actuator devices.
   * @param steering Handle for steering force calculator.
   */
  actuation_subsystem(const std::shared_ptr<rcppsw::er::server>& server,
                      const struct params::actuation_params* c_params,
                      struct actuator_list* list);

  /**
   * @brief Set the color of the robot's LEDs.
   *
   * @param color The new color.
   */
  void leds_set_color(const utils::color& color);

  throttling_differential_drive& differential_drive(void) { return m_drive; }
  const throttling_differential_drive& differential_drive(void) const {
    return m_drive;
  }

  /**
   * @brief Set whether or not temporary throttling of overall maximum speed is
   * enabled when a robot is carrying a block.
   */
  void block_throttle_toggle(bool en) { m_drive.throttle_toggle(en); }

  /**
   * @brief Update the currently applied amount of throttling based on
   * presumably new configuration.
   */
  void block_throttle_update(void) { m_drive.throttling_update(); }

  /**
   * @brief Reset the actuations, including stopping the robot.
   */
  void reset(void);

 private:
  // clang-format off
  const struct params::actuation_params    mc_params;
  struct actuator_list                     m_actuators;
  throttling_differential_drive            m_drive;
  // clang-format on
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_ACTUATION_SUBSYSTEM_HPP_ */
