/**
 * @file sensing_subsystem.cpp
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
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/params/sensing_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
sensing_subsystem::sensing_subsystem(
    const struct params::sensing_params* const params,
    const struct sensor_list* const list)
    : mc_obstacle_delta(params->proximity.delta),
      mc_los_dim(params->los_dim),
      m_tick(0),
      m_position(),
      m_prev_position(),
      m_sensors(*list),
      m_fov(rmath::radians(-5 * M_PI / 6), rmath::radians(5 * M_PI / 6)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool sensing_subsystem::in_nest(void) const {
  /*
   * The nest is a relatively light gray, so the sensors will return something
   * in the range specified below.
   *
   * They return 1.0 when the robot is on a white area, it is 0.0 when the robot
   * is on a black area.
   */
  return m_sensors.ground.detect(0.7, 0.1, 3);
} /* in_nest() */

rmath::vector2d sensing_subsystem::find_closest_obstacle(void) const {
  return m_sensors.proximity.closest_prox_obj(position(),
                                              mc_obstacle_delta,
                                              m_fov);
} /* find_closest_obstacle() */

bool sensing_subsystem::threatening_obstacle_exists(void) const {
  return m_sensors.proximity.prox_obj_exists(position(),
                                             mc_obstacle_delta,
                                             m_fov);
} /* threatening_obstacle_exists() */

bool sensing_subsystem::block_detected(void) const {
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
  return m_sensors.ground.detect(0.0, 0.05, 4);
} /* block_detected() */

bool sensing_subsystem::cache_detected(void) const {
  /*
   * We are on a cache if at least 3 of the 4 ground sensors say we are. Caches
   * are a relatively dark gray, so the sensor should return something in the
   * range specified below.
   */
  return m_sensors.ground.detect(0.4, 0.1, 3);
} /* block_detected() */

std::vector<std::vector<uint8_t>> sensing_subsystem::recieve_message() {
  std::vector<std::vector<uint8_t>> return_data;
  auto readings = m_sensors.rabs.readings();
  for (auto reading : readings) {
    std::vector<uint8_t> data = reading.data;
    // Check if data isn't empty and if there is a value in the state position
    if (!data.empty() && static_cast<int>(data[2]) >= 1 &&
      static_cast<int>(data[0]) > 0 && static_cast<int>(data[1]) > 0) {
      return_data.push_back(data);
    }
  }
  return return_data;
}

NS_END(controller, fordyca);
