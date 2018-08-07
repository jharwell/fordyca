/**
 * @file depth1/sensing_subsystem.cpp
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
#include "fordyca/controller/depth1/sensing_subsystem.hpp"
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
sensing_subsystem::sensing_subsystem(
    const struct params::sensing_params* c_params,
    const struct base_sensing_subsystem::sensor_list* const list)
    : depth0::sensing_subsystem(c_params, list) {}

bool sensing_subsystem::cache_detected(void) const {
  std::vector<hal::sensors::ground_sensor::reading> readings =
      base_sensing_subsystem::ground().readings();

  /*
   * We are on a cache if at least 3 of the 4 ground sensors say we are. Caches
   * are a relatively dark gray, so the sensor should return something in the
   * range specified below.
   */
  int sum = 0;
  sum += static_cast<int>(readings[0].value > 0.30 && readings[0].value < 0.50);
  sum += static_cast<int>(readings[1].value > 0.30 && readings[1].value < 0.50);
  sum += static_cast<int>(readings[2].value > 0.30 && readings[2].value < 0.50);
  sum += static_cast<int>(readings[3].value > 0.30 && readings[3].value < 0.50);

  return sum >= 3;
} /* block_detected() */

NS_END(depth1, controller, fordyca);
