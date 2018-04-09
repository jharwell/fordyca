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
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/datatypes/color.h>

#include "fordyca/params/actuation_params.hpp"
#include "fordyca/controller/throttling_handler.hpp"
#include "fordyca/controller/steering_force2D.hpp"
#include "fordyca/controller/footbot_differential_drive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace argos {
class CCI_DifferentialSteeringActuator;
class CCI_LEDsActuator;
class CCI_RangeAndBearingActuator;
} // namespace argos

NS_START(fordyca, controller);

namespace state_machine = rcppsw::patterns::state_machine;

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
    argos::CCI_DifferentialSteeringActuator* wheels;
    argos::CCI_LEDsActuator* leds;
    argos::CCI_RangeAndBearingActuator* raba;
  };

  /**
   * @brief Initialize the actuation subsystem.
   *
   * @param c_params Subsystem parameters.
   * @param list List of handles to actuator devices.
   * @param steering Handle for steering force calculator.
   */
  actuation_subsystem(
      const std::shared_ptr<rcppsw::er::server>& server,
      const struct params::actuation_params* c_params,
      struct actuator_list * list);

  /**
   * @brief Set the color of the robot's LEDs.
   *
   * @param color The new color.
   */
  void leds_set_color(const argos::CColor& color);

  footbot_differential_drive& differential_drive(void) { return m_drive; }
  const footbot_differential_drive& differential_drive(void) const { return m_drive; }

  void set_rel_heading(const argos::CVector2& heading,
                       bool force_hard_turn = false);
  /**
   * @brief Set whether or not temporary throttling of overall maximum speed is
   * enabled when a robot is carrying a block.
   */
  void block_throttle_toggle(bool en) { m_throttling.carrying_block(en); }

  /**
   * @brief Update the currently applied amount of throttling based on
   * presumably new configuration.
   */
  void block_throttle_update(void) { m_throttling.update(); }

  /**
   * @brief Reset the actuations, including stopping the robot.
   */
  void reset(void);

 private:
  // clang-format off
  const struct params::actuation_params    mc_params;
  struct actuator_list                     m_actuators;
  throttling_handler                       m_throttling;
  footbot_differential_drive               m_drive;
  // clang-format on
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_ACTUATION_SUBSYSTEM_HPP_ */
