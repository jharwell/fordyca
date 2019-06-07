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
#include "fordyca/config/actuation_config.hpp"
#include "fordyca/controller/actuator_list.hpp"
#include "rcppsw/robotics/steer2D/force_calculator.hpp"
#include "fordyca/controller/throttling_differential_drive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace support { namespace tv {
class tv_manager;
}} // namespace support::tv

NS_START(controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class actuation_subsystem
 * @ingroup fordyca controller
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
  /**
   * @brief Initialize the actuation subsystem.
   *
   * @param c_config Subsystem parameters.
   * @param list List of handles to actuator devices.
   */
  actuation_subsystem(const config::actuation_config* c_config,
                      struct actuator_list* list);

  /**
   * @brief Set the color of the robot's LEDs.
   *
   * @param color The new color.
   */
  void leds_set_color(const rutils::color& color);

  throttling_differential_drive& differential_drive(void) { return m_drive; }
  const throttling_differential_drive& differential_drive(void) const {
    return m_drive;
  }

  /**
   * @brief Reset the actuations, including stopping the robot.
   */
  void reset(void);

  void start_sending_message(struct hal::wifi_packet packet) {
    m_actuators.wifi.broadcast_start(packet);
  }

  void stop_sending_message(void) {
    m_actuators.wifi.broadcast_stop();
  }


 private:
  /* clang-format off */
  const config::actuation_config mc_config;
  struct actuator_list           m_actuators;
  throttling_differential_drive  m_drive;
  /* clang-format on */
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_ACTUATION_SUBSYSTEM_HPP_ */
