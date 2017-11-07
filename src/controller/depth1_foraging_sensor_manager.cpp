/**
 * @file depth1_foraging_sensor_manager.cpp
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
#include "fordyca/controller/depth1_foraging_sensor_manager.hpp"
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
depth1_foraging_sensor_manager::depth1_foraging_sensor_manager(
    const struct params::sensor_params* params,
    argos::CCI_RangeAndBearingSensor* const rabs,
    argos::CCI_FootBotProximitySensor* const proximity,
    argos::CCI_FootBotLightSensor* const light,
    argos::CCI_FootBotMotorGroundSensor* const ground) :
    depth0_foraging_sensor_manager(params, rabs, proximity, light, ground) {}

bool depth1_foraging_sensor_manager::cache_detected(void) {
  const argos::CCI_FootBotMotorGroundSensor::TReadings& readings =
      base_foraging_sensor_manager::ground()->GetReadings();
  int sum = 0;

  /*
   * We are on a cache if at least 3 of the 4 ground sensors say we are. Caches
   * are a relatively dark gray, so the sensor should return something in the
   * range specified below.
   */
  sum += readings[0].Value > 0.30 && readings[0].Value < 0.50;
  sum += readings[1].Value > 0.30 && readings[1].Value < 0.50;
  sum += readings[2].Value > 0.30 && readings[2].Value < 0.50;
  sum += readings[3].Value > 0.30 && readings[3].Value < 0.50;

  return sum >= 3;
} /* block_detected() */

NS_END(controller, fordyca);
