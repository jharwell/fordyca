/**
 * @file actuation_subsystem.cpp
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
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/params/actuation_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
using kinematics2D::differential_drive;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
actuation_subsystem::actuation_subsystem(
    const struct params::actuation_params* c_params,
    struct actuator_list* const list)
    : mc_params(*c_params),
      m_actuators(*list),
      m_drive(differential_drive::kFSMDrive,
              c_params->differential_drive.max_speed,
              c_params->differential_drive.soft_turn_max,
              m_actuators.wheels,
              &c_params->throttling) {}

void actuation_subsystem::reset(void) {
#ifdef FORDYCA_WITH_ROBOT_RAB
  m_actuators.wifi.reset();
#endif
  m_drive.stop();
} /* reset() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void actuation_subsystem::leds_set_color(__rcsw_unused const utils::color& color) {
#ifdef FORDYCA_WITH_ROBOT_LEDS
  m_actuators.leds.set_color(-1, color);
#endif
} /* leds_set_color() */

NS_END(controller, fordyca);
