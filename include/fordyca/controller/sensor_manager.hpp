/**
 * @file sensor_manager.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_SENSOR_MANAGER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_SENSOR_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include <argos3/core/utility/math/vector2.h>
#include "rcppsw/common/common.hpp"
#include "fordyca/representation/line_of_sight.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);


namespace params {
struct sensor_params;
} /* namespace params */

NS_START(controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class sensor_manager {
 public:
  sensor_manager(
      const struct params::sensor_params* params,
      argos::CCI_RangeAndBearingSensor* const rabs,
      argos::CCI_FootBotProximitySensor* const proximity,
      argos::CCI_FootBotLightSensor* const light,
      argos::CCI_FootBotMotorGroundSensor* const ground);

  const argos::CCI_RangeAndBearingSensor::TReadings& range_and_bearing(void) const {
    return m_rabs->GetReadings();
  }
  const argos::CCI_FootBotMotorGroundSensor::TReadings& ground(void) const {
    return m_ground->GetReadings();
  }

  /**
   * @brief If TRUE, a block has *possibly* been detected.
   *
   * Only possibly, because there are some false positives, such as the first
   * timestep, before ARGoS has finished initializing things.
   */
  bool block_detected(void);

  /**
   * @brief If TRUE, a block has *possibly* been detected.
   *
   * Only possibly, because there are some false positives, such as the first
   * timestep, before ARGoS has finished initializing things.
   */
  bool cache_detected(void);

  /**
   * @brief If TRUE, the robot is currently in the nest, as reported by 3/4 of
   * its ground sensors.
   */
  bool in_nest(void);

  /**
   * @brief Get the robot's current line-of-sight (LOS)
   */
  const representation::line_of_sight* los(void) const { return m_los.get(); }

  /**
   * @brief This is a hack to make it easy for me to run simulations, as I can
   * computer the line of sight for a robot within the loop functions, and just
   * pass it in here. In real robots this routine would be MUCH messier and
   * harder to work with.
   *
   * @param los The new los
   */
  void los(std::unique_ptr<representation::line_of_sight>& los) {
    m_los = std::move(los);
  }

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

  /**
   * @brief Get the current simulation time tick.
   */
  uint tick(void) const { return m_tick; }

  /**
   * @brief Set the current simulation time tick.
   */
  void tick(uint tick) { m_tick = tick; }

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
   * @brief Calcucate the location of the light within the arena...not sure when
   * I would ever need to use this...
   */
  argos::CVector2 calc_light_loc(const argos::CVector2& robot_loc);

 private:
  sensor_manager(const sensor_manager& fsm) = delete;
  sensor_manager& operator=(const sensor_manager& fsm) = delete;

  /** The current timestep  */
  uint                                        m_tick;
  std::shared_ptr<const struct params::sensor_params> mc_params;
  argos::CCI_RangeAndBearingSensor*           m_rabs; /* range and bearing sensor */
  argos::CCI_FootBotProximitySensor*          m_proximity; /* proximity sensor */
  argos::CCI_FootBotLightSensor*              m_light; /* light sensor */
  argos::CCI_FootBotMotorGroundSensor*        m_ground; /* motor ground sensor */
  std::unique_ptr<representation::line_of_sight> m_los;
  argos::CVector2                             m_robot_loc;
  argos::CVector2                             m_prev_robot_loc;
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_SENSOR_MANAGER_HPP_ */
