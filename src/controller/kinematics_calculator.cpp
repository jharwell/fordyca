/**
 * @file kinematics_calculator.cpp
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
#include "fordyca/controller/kinematics_calculator.hpp"
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/base_sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
argos::CVector2 kinematics_calculator::calc_avoidance_force(void) {
  argos::CVector2 closest = m_sensors->find_closest_obstacle();
  argos::CVector2 avoidance;

  if (closest.Length() > 0) {
    avoidance = -closest;
    avoidance.Normalize();
    avoidance *= kSCALE_AVOIDANCE_FORCE * mc_actuators->differential_drive()->max_speed();
  } else {
    avoidance.Scale(0.0, 0.0);
  }
  return avoidance;
} /* calc_obstacle_vector() */

argos::CVector2 kinematics_calculator::calc_light_attract_force(void) {
  argos::CVector2 accum;

  for (auto& r : m_sensors->light()->GetReadings()) {
    accum += argos::CVector2(r.Value, r.Angle);
  } /* for(r..) */

  return argos::CVector2(1.0, accum.Angle()) * kSCALE_LIGHT_FORCE_ATTRACT *
      mc_actuators->differential_drive()->max_speed();
} /* calc_light_attract_force() */

argos::CVector2 kinematics_calculator::calc_light_repel_force(void) {
  argos::CVector2 accum;

  for (auto& r : m_sensors->light()->GetReadings()) {
    accum += argos::CVector2(r.Value, r.Angle);
  } /* for(r..) */

  return -argos::CVector2(1.0, accum.Angle()) * kSCALE_LIGHT_FORCE_REPEL *
      mc_actuators->differential_drive()->max_speed();
} /* calc_light_repel_force() */

NS_END(controller, fordyca);
