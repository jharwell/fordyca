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

#ifndef INCLUDE_FORDYCA_SENSOR_MANAGER_HPP_
#define INCLUDE_FORDYCA_SENSOR_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include <argos3/core/utility/math/vector2.h>
#include "rcppsw/common/common.hpp"
#include "fordyca/params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class sensor_manager {
 public:
  /* constructors */
  sensor_manager(
      const struct sensor_params* params,
      argos::CCI_RangeAndBearingSensor* const rabs,
      argos::CCI_FootBotProximitySensor* const proximity,
      argos::CCI_FootBotLightSensor* const light,
      argos::CCI_FootBotMotorGroundSensor* const ground);

  /* member functions */
  const argos::CCI_RangeAndBearingSensor::TReadings& range_and_bearing(void) {
    return m_rabs->GetReadings();
  }
  const argos::CCI_FootBotMotorGroundSensor::TReadings& ground(void) {
    return m_ground->GetReadings();
  }

  void update_position(argos::CVector2& new_pos);

  bool detect_food_item(void);
  bool in_nest(void);
  /*
   * Calculates the diffusion vector. If there is a close obstacle, it points
   * away from it; it there is none, it points forwards.  The b_collision
   * parameter is used to return true or false whether a collision avoidance
   * just happened or not. It is necessary for the collision rule.
   */
  bool calc_diffusion_vector(argos::CVector2* const vector);
  /*
   * Calculates the vector to the light. Used to perform
   * phototaxis and antiphototaxis.
   */
  argos::CVector2 calc_vector_to_light(void);

 private:
  sensor_manager(const sensor_manager& fsm) = delete;
  sensor_manager& operator=(const sensor_manager& fsm) = delete;

  std::shared_ptr<const struct sensor_params>          mc_params;
  argos::CCI_RangeAndBearingSensor*    m_rabs; /* range and bearing sensor */
  argos::CCI_FootBotProximitySensor*   m_proximity; /* proximity sensor */
  argos::CCI_FootBotLightSensor*       m_light; /* light sensor */
  argos::CCI_FootBotMotorGroundSensor* m_ground; /* motor ground sensor */
};

NS_END(fordyca);

#endif /* INCLUDE_FORDYCA_SENSOR_MANAGER_HPP_ */
