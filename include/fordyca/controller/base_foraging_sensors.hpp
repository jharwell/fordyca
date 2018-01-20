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
/**
 * @class base_foraging_sensors
 * @ingroup controller
 *
 * @brief The base sensor class for all sensors used by the different foraging
 * controllers. Contains common functionality to all sensors.
 */
class base_foraging_sensors {
 public:
  base_foraging_sensors(
      const struct params::sensor_params* params,
      argos::CCI_RangeAndBearingSensor* rabs,
      argos::CCI_FootBotProximitySensor* proximity,
      argos::CCI_FootBotLightSensor* light,
      argos::CCI_FootBotMotorGroundSensor* ground);

  base_foraging_sensors(double diffusion_delta,
                        argos::CRange<argos::CRadians> go_straight_angle_range,
                        argos::CCI_RangeAndBearingSensor* rabs,
                        argos::CCI_FootBotProximitySensor* proximity,
                        argos::CCI_FootBotLightSensor* light,
                        argos::CCI_FootBotMotorGroundSensor* ground);

  base_foraging_sensors(const base_foraging_sensors& fsm) = default;
  base_foraging_sensors& operator=(const base_foraging_sensors& fsm) = delete;

  /**
   * @brief If \c TRUE, a block has *possibly* been detected.
   *
   * Only possibly, because there are some false positives, such as the first
   * timestep, before ARGoS has finished initializing things.
   */
  bool block_detected(void);

  /**
   * @brief If \c TRUE, the robot is currently in the nest, as reported by 3/4
   * of its ground sensors.
   */
  bool in_nest(void);

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
   * @brief Get the current simulation time tick.
   */
  uint tick(void) const { return m_tick; }

  /**
   * @brief Set the current simulation time tick.
   */
  void tick(uint tick) { m_tick = tick; }

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

  argos::CCI_RangeAndBearingSensor* rabs(void) const { return m_rabs; }
  argos::CCI_FootBotProximitySensor* proximity(void) const { return m_proximity; }
  argos::CCI_FootBotLightSensor* light(void) const { return m_light; }
  argos::CCI_FootBotMotorGroundSensor* ground(void) const { return m_ground; }
  double diffusion_delta(void) const { return mc_obstacle_delta; }

  /**
   * @brief Figure out if a threatening obstacle exists near to the robot's
   * current location.
   *
   * A threatening obstacle is defined as one that is closer than the defined
   * obstacle delta to the robot. Note that the obstacle delta is NOT a measure
   * of distance, but a measure [0, 1] indicating how close an obstacle is which
   * increases exponentially as the obstacle nears.
   *
   * @return \c TRUE if a threatening obstacle is found, \c FALSE otherwise.
   */
  bool threatening_obstacle_exists(void);

  /**
   * @brief Return the closest obstacle (i.e. the most threatening).
   *
   * Should be used in conjunction with \ref threatening_obstacle_exists().
   */
  argos::CVector2 find_closest_obstacle(void);

 private:
  /**
   * @brief Determine if the obstacle represented by its closest point to the
   * robot is threatening.
   */
  bool obstacle_is_threatening(const argos::CVector2& obstacle);

  // clang-format off
  uint                                        m_tick;
  const double                                mc_obstacle_delta;
  argos::CVector2                             m_robot_loc;
  argos::CVector2                             m_prev_robot_loc;
  const argos::CRange<argos::CRadians>        mc_go_straight_angle_range;
  argos::CCI_RangeAndBearingSensor*           m_rabs;
  argos::CCI_FootBotProximitySensor*          m_proximity;
  argos::CCI_FootBotLightSensor*              m_light;
  argos::CCI_FootBotMotorGroundSensor*        m_ground;
  // clang-format off
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_BASE_FORAGING_SENSORS_HPP_ */
