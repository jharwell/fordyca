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
#include "fordyca/sensor_manager.hpp"
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
sensor_manager::sensor_manager(
    const struct sensor_params* params,
    argos::CCI_RangeAndBearingSensor* const rabs,
    argos::CCI_FootBotProximitySensor* const proximity,
    argos::CCI_FootBotLightSensor* const light,
    argos::CCI_FootBotMotorGroundSensor* const ground) :
    mc_params(params),
    m_rabs(rabs),
    m_proximity(proximity),
    m_light(light),
    m_ground(ground) {}


/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool sensor_manager::in_nest(void) {
  /* Read stuff from the ground sensor */
  const argos::CCI_FootBotMotorGroundSensor::TReadings& tGroundReads = m_ground->GetReadings();

  /*
   * You can say whether you are in the nest by checking the ground sensor
   * placed close to the wheel motors. It returns a value between 0 and 1.  It
   * is 1 when the robot is on a white area, it is 0 when the robot is on a
   * black area and it is around 0.5 when the robot is on a gray area.
   *
   * The foot-bot has 4 sensors like this, two in the front (corresponding to
   * readings 0 and 1) and two in the back (corresponding to reading 2 and 3).
   * Here we want the back sensors (readings 2 and 3) to tell us whether we are
   * on gray: if so, the robot is completely in the nest, otherwise it's
   * outside.
   */
  if (tGroundReads[2].Value > 0.25f && tGroundReads[2].Value < 0.75f &&
      tGroundReads[3].Value > 0.25f && tGroundReads[3].Value < 0.75f) {
    return true;
  }
  return false;
} /* in_nest() */

bool sensor_manager::calc_diffusion_vector(argos::CVector2* const vector_in) {
  /* Get readings from proximity sensor */
  const argos::CCI_FootBotProximitySensor::TReadings& tProxReads = m_proximity->GetReadings();
  argos::CVector2* vector;
  argos::CVector2 tmp;
  if (vector_in == NULL) {
    vector = &tmp;
  } else {
    vector = vector_in;
  }

  /* Sum them together */
  for (size_t i = 0; i < tProxReads.size(); ++i) {
    *vector += argos::CVector2(tProxReads[i].Value, tProxReads[i].Angle);
  }
  /*
   * If the angle of the vector is small enough and the closest obstacle is far
   * enough, ignore the vector and go straight, otherwise return it.
   */
  if(mc_params->diffusion.go_straight_angle_range.WithinMinBoundIncludedMaxBoundIncluded(vector->Angle()) &&
     vector->Length() < mc_params->diffusion.delta) {
    *vector = argos::CVector2::X;
    return false;
  }
  *vector = -vector->Normalize();
  return true;
}

argos::CVector2 sensor_manager::calc_vector_to_light(void) {
  /* Get readings from light sensor */
  const argos::CCI_FootBotLightSensor::TReadings& tLightReads = m_light->GetReadings();
  /* Sum them together */
  argos::CVector2 accum;
  for(size_t i = 0; i < tLightReads.size(); ++i) {
    accum += argos::CVector2(tLightReads[i].Value, tLightReads[i].Angle);
  }
  /* If the light was perceived, return the vector. Otherwise return 0. */
  if(accum.Length() > 0.0f) {
    return argos::CVector2(1.0f, accum.Angle());
  } else {
    return argos::CVector2();
  }
} /* calc_vector_to_light() */

bool sensor_manager::detect_food_item(void) {
  const argos::CCI_FootBotMotorGroundSensor::TReadings& readings = m_ground->GetReadings();
  return false;
} /* sensor_manager:detect_food_item() */


NS_END(fordyca);
