/**
 * @file sensor_manager.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/sensor_manager.hpp"
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
sensor_manager::sensor_manager(
    const struct sensor_params* params,
    argos::CCI_RangeAndBearingSensor* const rabs,
    argos::CCI_FootBotProximitySensor* const proximity,
    argos::CCI_FootBotLightSensor* const light,
    argos::CCI_FootBotMotorGroundSensor* const ground) :
    m_tick(0),
    mc_params(params),
    m_rabs(rabs),
    m_proximity(proximity),
    m_light(light),
    m_ground(ground),
    m_los(),
    m_robot_loc(),
    m_prev_robot_loc() {}


/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool sensor_manager::in_nest(void) {
  /* Read stuff from the ground sensor */
  const argos::CCI_FootBotMotorGroundSensor::TReadings& readings = m_ground->GetReadings();
  /*
   * You can say whether you are in the nest by checking the ground sensor
   * placed close to the wheel motors. It returns a value between 0 and 1.  It
   * is 1 when the robot is on a white area, it is 0 when the robot is on a
   * black area and it is around 0.5 when the robot is on a gray area.
   */
  if (readings[0].Value > 0.25f && readings[0].Value < 0.75f &&
      readings[1].Value > 0.25f && readings[1].Value < 0.75f &&
      readings[2].Value > 0.25f && readings[2].Value < 0.75f &&
      readings[3].Value > 0.25f && readings[3].Value < 0.75f) {
    return true;
  }
  return false;
} /* in_nest() */

bool sensor_manager::calc_diffusion_vector(argos::CVector2* const vector_in) {
  const argos::CCI_FootBotProximitySensor::TReadings& tProxReads = m_proximity->GetReadings();
  argos::CVector2* vector;
  argos::CVector2 tmp;

  if (vector_in == NULL) {
    vector = &tmp;
  } else {
    vector = vector_in;
  }

  for (size_t i = 0; i < tProxReads.size(); ++i) {
    *vector += argos::CVector2(tProxReads[i].Value, tProxReads[i].Angle);
  } /* for(i..) */

  /*
   * If the angle of the vector is small enough and the closest obstacle is far
   * enough, ignore the vector and go straight, otherwise return it.
   */
  if (mc_params->diffusion.go_straight_angle_range.WithinMinBoundIncludedMaxBoundIncluded(vector->Angle()) &&
     vector->Length() < mc_params->diffusion.delta) {
    *vector = argos::CVector2::X;
    return false;
  }
  *vector = -vector->Normalize();
  return true;
}

argos::CVector2 sensor_manager::calc_vector_to_light(void) {
  const argos::CCI_FootBotLightSensor::TReadings& tLightReads = m_light->GetReadings();
  argos::CVector2 accum;

  for (size_t i = 0; i < tLightReads.size(); ++i) {
    accum += argos::CVector2(tLightReads[i].Value, tLightReads[i].Angle);
  } /* for(i..) */

  return argos::CVector2(1.0f, accum.Angle());
} /* calc_vector_to_light() */

argos::CVector2 sensor_manager::calc_light_loc(const argos::CVector2& robot_loc) {
  argos::CVector2 robot_to_light_vec = calc_vector_to_light();
  return argos::CVector2(std::fabs(robot_loc.GetX() - robot_to_light_vec.GetX()),
                         std::fabs(robot_loc.GetY() - robot_to_light_vec.GetY()));
} /* calc_light_loc() */


bool sensor_manager::block_detected(void) {
  const argos::CCI_FootBotMotorGroundSensor::TReadings& readings = m_ground->GetReadings();
  int sum = 0;

  /* We are on a block if at least 3 of the 4 ground sensors say we are */
  sum += readings[0].Value < 0.05;
  sum += readings[1].Value < 0.05;
  sum += readings[2].Value < 0.05;
  sum += readings[3].Value < 0.05;

  return sum >= 3;
} /* block_detected() */

NS_END(controller, fordyca);
