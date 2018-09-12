/**
 * @file saa_subsystem.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/controller/saa_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
namespace kinematics = rcppsw::robotics::kinematics;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
saa_subsystem::saa_subsystem(
    const struct params::actuation_params* const aparams,
    const struct params::sensing_params* const sparams,
    struct actuation_subsystem::actuator_list* const actuator_list,
    struct base_sensing_subsystem::sensor_list* const sensor_list)
    : ER_CLIENT_INIT("fordyca.controller.saa_subsystem"),
      m_actuation(std::make_shared<actuation_subsystem>(aparams, actuator_list)),
      m_sensing(std::make_shared<base_sensing_subsystem>(sparams, sensor_list)),
      m_steering(*this, &aparams->steering, m_sensing) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void saa_subsystem::apply_steering_force(const std::pair<bool, bool>& force) {
  ER_DEBUG("position=(%f, %f)",
           m_sensing->position().GetX(),
           m_sensing->position().GetY())
  ER_DEBUG("linear_vel=(%f,%f)@%f [%f] angular_vel=%f",
           linear_velocity().GetX(),
           linear_velocity().GetY(),
           linear_velocity().Angle().GetValue(),
           linear_velocity().Length(),
           angular_velocity());
  ER_DEBUG("steering_force=(%f,%f)@%f [%f]",
           m_steering.value().GetX(),
           m_steering.value().GetY(),
           m_steering.value().Angle().GetValue(),
           m_steering.value().Length());

  m_actuation->differential_drive().fsm_drive(m_steering.value().Length(),
                                              m_steering.value().Angle(),
                                              force);
  m_steering.reset();
}
NS_END(controller, fordyca);
