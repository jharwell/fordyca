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
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/repr/line_of_sight.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
saa_subsystem::saa_subsystem(const config::actuation_config* const aconfig,
                             const config::sensing_config* const sconfig,
                             actuator_list* const actuator_list,
                             sensor_list* const sensor_list)
    : ER_CLIENT_INIT("fordyca.controller.saa_subsystem"),
      m_actuation(std::make_shared<actuation_subsystem>(aconfig, actuator_list)),
      m_sensing(std::make_shared<sensing_subsystem>(sconfig, sensor_list)),
      m_steer2D_calc(*this, &aconfig->steering) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void saa_subsystem::steer2D_force_apply(const std::pair<bool, bool>& force) {
  ER_DEBUG("position=(%f, %f)",
           m_sensing->position().x(),
           m_sensing->position().y())
  ER_DEBUG("linear_vel=(%f,%f)@%f [%f] angular_vel=%f",
           linear_velocity().x(),
           linear_velocity().y(),
           linear_velocity().angle().value(),
           linear_velocity().length(),
           angular_velocity());
  ER_DEBUG("steering_force=(%f,%f)@%f [%f]",
           m_steer2D_calc.value().x(),
           m_steer2D_calc.value().y(),
           m_steer2D_calc.value().angle().value(),
           m_steer2D_calc.value().length());

  double speed = m_steer2D_calc.value().length() *
                 (1.0 - m_actuation->differential_drive().active_throttle());
  m_actuation->differential_drive().fsm_drive(speed,
                                              m_steer2D_calc.value().angle(),
                                              force);
  m_steer2D_calc.reset();
} /* steer2D_force_apply() */

rmath::vector2d saa_subsystem::linear_velocity(void) const {
  return {m_actuation->differential_drive().current_speed(),
          m_sensing->heading().angle()};
} /* linear_velocity() */

__rcsw_pure double saa_subsystem::angular_velocity(void) const {
  return (m_actuation->differential_drive().right_linspeed() -
          m_actuation->differential_drive().left_linspeed()) /
         m_actuation->differential_drive().axle_length();
} /* angular_velocity() */

__rcsw_pure double saa_subsystem::max_speed(void) const {
  return m_actuation->differential_drive().max_speed();
} /* max_speed() */

__rcsw_pure rmath::vector2d saa_subsystem::position(void) const {
  return m_sensing->position();
} /* position() */
NS_END(controller, fordyca);
