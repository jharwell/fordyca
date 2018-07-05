/**
 * @file base_sensing_subsystem.cpp
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
#include "fordyca/controller/base_sensing_subsystem.hpp"
#include <limits>

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include "fordyca/params/sensing_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_sensing_subsystem::base_sensing_subsystem(
    const struct params::sensing_params* const params,
    const struct sensor_list* const list)
    : m_tick(0),
      mc_obstacle_delta(params->proximity.delta),
      m_position(),
      m_prev_position(),
      m_sensors(*list) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool base_sensing_subsystem::in_nest(void) {
  std::vector<hal::sensors::ground_sensor::reading> readings =
      m_sensors.ground.readings();
  /*
   * The nest is a relatively light gray, so the sensors will return something
   * in the range specified below.
   *
   * They return 1.0 when the robot is on a white area, it is 0.0 when the robot
   * is on a black area.
   */
  int sum = 0;
  sum += static_cast<int>(readings[0].value > 0.60 && readings[0].value < 0.80);
  sum += static_cast<int>(readings[1].value > 0.60 && readings[1].value < 0.80);
  sum += static_cast<int>(readings[2].value > 0.60 && readings[2].value < 0.80);
  sum += static_cast<int>(readings[3].value > 0.60 && readings[3].value < 0.80);

  return sum >= 3;
} /* in_nest() */

__rcsw_pure bool base_sensing_subsystem::obstacle_is_threatening(
    const argos::CVector2& obstacle) const {
  argos::CRange<argos::CRadians> range(argos::CRadians(-5 * M_PI / 6),
                                       argos::CRadians(5 * M_PI / 6));
  return obstacle.Length() >= mc_obstacle_delta &&
         range.WithinMinBoundIncludedMaxBoundIncluded(obstacle.Angle());
} /* obstacle_is_threatening() */

argos::CVector2 base_sensing_subsystem::find_closest_obstacle(void) const {
  std::pair<argos::CVector2, bool> res;
  argos::CVector2 closest(0, 0);

  for (auto& r : m_sensors.proximity.readings()) {
    argos::CVector2 obstacle(r.value, argos::CRadians(r.angle));
    if (obstacle_is_threatening(obstacle)) {
      if ((position() - obstacle).Length() < closest.Length() ||
          closest.Length() <= std::numeric_limits<double>::epsilon()) {
        closest = obstacle;
      }
    }
  } /* for(r..) */
  return closest;
} /* find_closest_obstacle() */

bool base_sensing_subsystem::threatening_obstacle_exists(void) const {
  return find_closest_obstacle().Length() > 0;
} /* threatening_obstacle_exists() */

bool base_sensing_subsystem::block_detected(void) {
  std::vector<hal::sensors::ground_sensor::reading> readings =
      m_sensors.ground.readings();
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
  sum += static_cast<int>(readings[0].value < 0.05);
  sum += static_cast<int>(readings[1].value < 0.05);
  sum += static_cast<int>(readings[2].value < 0.05);
  sum += static_cast<int>(readings[3].value < 0.05);

  return 4 == sum;
} /* block_detected() */

NS_END(controller, fordyca);
