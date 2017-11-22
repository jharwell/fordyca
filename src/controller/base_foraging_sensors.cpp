/**
 * @file base_foraging_sensors.cpp
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
#include "fordyca/controller/base_foraging_sensors.hpp"
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include "fordyca/params/sensor_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_foraging_sensors::base_foraging_sensors(
    const struct params::sensor_params* params,
    argos::CCI_RangeAndBearingSensor* const rabs,
    argos::CCI_FootBotProximitySensor* const proximity,
    argos::CCI_FootBotLightSensor* const light,
    argos::CCI_FootBotMotorGroundSensor* const ground) :
    mc_diffusion_delta(params->diffusion.delta),
    m_robot_loc(),
    m_prev_robot_loc(),
    mc_go_straight_angle_range(params->diffusion.go_straight_angle_range),
    m_rabs(rabs),
    m_proximity(proximity),
    m_light(light),
    m_ground(ground) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool base_foraging_sensors::in_nest(void) {
  const argos::CCI_FootBotMotorGroundSensor::TReadings& readings = m_ground->GetReadings();
  /*
   * The nest is a relatively light gray, so the sensors will return something
   * in the range specified below.
   *
   * They return 1.0 when the robot is on a white area, it is 0.0 when the robot
   * is on a black area.
   */
  int sum = 0;
  sum += readings[0].Value > 0.60 && readings[0].Value < 0.80;
  sum += readings[1].Value > 0.60 && readings[1].Value < 0.80;
  sum += readings[2].Value > 0.60 && readings[2].Value < 0.80;
  sum += readings[3].Value > 0.60 && readings[3].Value < 0.80;

  return sum >= 3;
} /* in_nest() */

bool base_foraging_sensors::calc_diffusion_vector(argos::CVector2* const vector_in) {
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
  if (mc_go_straight_angle_range.WithinMinBoundIncludedMaxBoundIncluded(vector->Angle()) &&
     vector->Length() < mc_diffusion_delta) {
    *vector = argos::CVector2::X;
    return false;
  }
  *vector = -vector->Normalize();
  return true;
}

argos::CVector2 base_foraging_sensors::calc_vector_to_light(void) {
  const argos::CCI_FootBotLightSensor::TReadings& tLightReads = m_light->GetReadings();
  argos::CVector2 accum;

  for (size_t i = 0; i < tLightReads.size(); ++i) {
    accum += argos::CVector2(tLightReads[i].Value, tLightReads[i].Angle);
  } /* for(i..) */

  return argos::CVector2(1.0, accum.Angle());
} /* calc_vector_to_light() */

bool base_foraging_sensors::block_detected(void) {
  const argos::CCI_FootBotMotorGroundSensor::TReadings& readings = m_ground->GetReadings();
  int sum = 0;

  /*
   * We are on a block fif ALL 4 ground sensors say we are. We can usually get
   * by with 3/4, but sometimes there are corner cases where a robot thinks that
   * it has arrived at a block, and is waiting for the pickup signal from the
   * simulation, but the simulation does not think that the robot is actually on
   * the block (i.e. it is only partially overlapping, with the center of the
   * robot being juusstttt outside the area of the block).
   *
   * Blocks are black, so sensors should return 0 when the robot is on a block.
   */
  sum += readings[0].Value < 0.05;
  sum += readings[1].Value < 0.05;
  sum += readings[2].Value < 0.05;
  sum += readings[3].Value < 0.05;

  return 4 == sum;
} /* block_detected() */

NS_END(controller, fordyca);
