/**
 * @file base_foraging_sensors.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_BASE_FORAGING_SENSORS_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_BASE_FORAGING_SENSORS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace argos {
class CCI_RangeAndBearingSensor;
class CCI_FootBotProximitySensor;
class CCI_FootBotLightSensor;
class CCI_FootBotMotorGroundSensor;
}
NS_START(fordyca);

namespace params { struct sensor_params; }

NS_START(controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class base_foraging_sensors {
 public:
  base_foraging_sensors(
      const struct params::sensor_params* params,
      argos::CCI_RangeAndBearingSensor* const rabs,
      argos::CCI_FootBotProximitySensor* const proximity,
      argos::CCI_FootBotLightSensor* const light,
      argos::CCI_FootBotMotorGroundSensor* const ground);

  /**
   * @brief If TRUE, a block has *possibly* been detected.
   *
   * Only possibly, because there are some false positives, such as the first
   * timestep, before ARGoS has finished initializing things.
   */
  bool block_detected(void);

  /**
   * @brief If TRUE, the robot is currently in the nest, as reported by 3/4 of
   * its ground sensors.
   */
  bool in_nest(void);

  /*
   * @brief Calculates the diffusion vector.
   *
   * If there is a close obstacle, it points away from it; it there is none, it
   * points forwards.  The b_collision parameter is used to return true or false
   * whether a collision avoidance just happened or not. It is necessary for the
   * collision rule.
   */
  bool calc_diffusion_vector(argos::CVector2* const vector);

  /*
   * Calculates the vector to the light. Used to perform
   * phototaxis and antiphototaxis.
   */
  argos::CVector2 calc_vector_to_light(void);

  /**
   * @brief Get the robot's current location.
   *
   * Note that this is set via loop functions, and that robots are not capable
   * of self-localizing. That's not the point of this project, and this was much
   * faster/easier.
   */
  argos::CVector2 robot_loc(void) const { return m_robot_loc; }

  /**
   * @brief Set the robot's current location.
   */
  void robot_loc(argos::CVector2 robot_loc) {
    m_prev_robot_loc = m_robot_loc;
    m_robot_loc = robot_loc;
  }


  /**
   * @brief Get the robot's heading, which is computed from the previous 2
   * calculated (ahem set) robot positions.
   */
  argos::CVector2 robot_heading(void) { return m_robot_loc - m_prev_robot_loc; }

  /**
   * @brief Get the angle of the current robot's heading. A shortcut to help
   * reduce the ache in my typing fingers.
   *
   * @return The heading angle.
   */
  argos::CRadians heading_angle(void) { return robot_heading().Angle(); }

 protected:
  argos::CCI_RangeAndBearingSensor* rabs(void) { return m_rabs; }
  argos::CCI_FootBotProximitySensor* proximity(void) { return m_proximity; }
  argos::CCI_FootBotLightSensor* light(void) { return m_light; }
  argos::CCI_FootBotMotorGroundSensor* ground(void) { return m_ground; }

 private:
  base_foraging_sensors(const base_foraging_sensors& fsm) = delete;
  base_foraging_sensors& operator=(const base_foraging_sensors& fsm) = delete;

  /** The current timestep  */
  const double                                mc_diffusion_delta;
  argos::CVector2                             m_robot_loc;
  argos::CVector2                             m_prev_robot_loc;
  const argos::CRange<argos::CRadians>        mc_go_straight_angle_range;
  argos::CCI_RangeAndBearingSensor*           m_rabs; /* range and bearing sensor */
  argos::CCI_FootBotProximitySensor*          m_proximity; /* proximity sensor */
  argos::CCI_FootBotLightSensor*              m_light; /* light sensor */
  argos::CCI_FootBotMotorGroundSensor*        m_ground; /* motor ground sensor */
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_BASE_FORAGING_SENSORS_HPP_ */
