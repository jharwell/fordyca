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
#include "fordyca/params/params.hpp"
#include "fordyca/representation/line_of_sight.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

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
  const argos::CCI_RangeAndBearingSensor::TReadings& range_and_bearing(void) const {
    return m_rabs->GetReadings();
  }
  const argos::CCI_FootBotMotorGroundSensor::TReadings& ground(void) const {
    return m_ground->GetReadings();
  }

  bool block_detected(void);
  bool in_nest(void);
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

  std::shared_ptr<const struct sensor_params> mc_params;
  argos::CCI_RangeAndBearingSensor*           m_rabs; /* range and bearing sensor */
  argos::CCI_FootBotProximitySensor*          m_proximity; /* proximity sensor */
  argos::CCI_FootBotLightSensor*              m_light; /* light sensor */
  argos::CCI_FootBotMotorGroundSensor*        m_ground; /* motor ground sensor */
  std::unique_ptr<representation::line_of_sight> m_los;
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_SENSOR_MANAGER_HPP_ */
