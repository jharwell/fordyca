/**
 * @file base_sensing_subsystem.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_BASE_SENSING_SUBSYSTEM_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_BASE_SENSING_SUBSYSTEM_HPP_

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
} // namespace argos

NS_START(fordyca);

namespace params {
struct sensing_params;
}

NS_START(controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class base_sensing_subsystem
 * @ingroup controller
 *
 * @brief The base sensing subsystem for all sensors used by the different
 * foraging controllers.  Contains common sensor functionality for all
 * controllers.
 */
class base_sensing_subsystem {
 public:
  struct sensor_list {
    argos::CCI_RangeAndBearingSensor* rabs;
    argos::CCI_FootBotProximitySensor* proximity;
    argos::CCI_FootBotLightSensor* light;
    argos::CCI_FootBotMotorGroundSensor* ground;
  };

  /**
   * @brief Initialize the base sensing subsystem.
   *
   * @param params Subsystem parameters.
   * @param list List of handles to sensing devices.
   */
  base_sensing_subsystem(const struct params::sensing_params* params,
                         const struct sensor_list* list);

  base_sensing_subsystem(double proximity_delta,
                         argos::CRange<argos::CRadians> go_straight_angle_range,
                         const struct sensor_list* list);

  base_sensing_subsystem(const base_sensing_subsystem& fsm) = default;
  base_sensing_subsystem& operator=(const base_sensing_subsystem& fsm) = delete;

  /**
   * @brief Get the list of sensors that the subsystem is managing.
   */
  const sensor_list& sensor_list(void) const { return m_sensors; }

  argos::CCI_RangeAndBearingSensor* rabs(void) const { return m_sensors.rabs; }
  argos::CCI_FootBotProximitySensor* proximity(void) const {
    return m_sensors.proximity;
  }
  argos::CCI_FootBotLightSensor* light(void) const { return m_sensors.light; }
  argos::CCI_FootBotMotorGroundSensor* ground(void) const {
    return m_sensors.ground;
  }

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
  argos::CVector2 position(void) const { return m_position; }

  /**
   * @brief Set the robot's current location.
   */
  void position(argos::CVector2 position) {
    m_prev_position = m_position;
    m_position = position;
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
  argos::CVector2 robot_heading(void) const {
    return m_position - m_prev_position;
  }

  /**
   * @brief Get the angle of the current robot's heading. A shortcut to help
   * reduce the ache in my typing fingers.
   *
   * @return The heading angle.
   */
  argos::CRadians heading_angle(void) const { return robot_heading().Angle(); }

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
  bool threatening_obstacle_exists(void) const;

  /**
   * @brief Return the closest obstacle (i.e. the most threatening).
   *
   * Should be used in conjunction with \ref threatening_obstacle_exists().
   */
  argos::CVector2 find_closest_obstacle(void) const;

 private:
  /**
   * @brief Determine if the obstacle represented by its closest point to the
   * robot is threatening.
   */
  bool obstacle_is_threatening(const argos::CVector2& obstacle) const;

  // clang-format off
  uint                                        m_tick;
  const double                                mc_obstacle_delta;
  argos::CVector2                             m_position;
  argos::CVector2                             m_prev_position;
  const argos::CRange<argos::CRadians>        mc_go_straight_angle_range;
  struct sensor_list                          m_sensors;
  // clang-format off
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_BASE_SENSING_SUBSYSTEM_HPP_ */
