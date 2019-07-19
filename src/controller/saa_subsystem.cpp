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
void saa_subsystem::steer2D_force_apply(void) {
  ER_DEBUG("position=%s heading=%s",
           m_sensing->position().to_str().c_str(),
           m_sensing->heading().to_str().c_str())
  ER_DEBUG("linear_vel=%s@%f [%f] angular_vel=%f",
           linear_velocity().to_str().c_str(),
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
                                              m_steer2D_calc.value().angle());
  m_steer2D_calc.reset();
} /* steer2D_force_apply() */

rmath::vector2d saa_subsystem::linear_velocity(void) const {
  auto speed = m_actuation->differential_drive().current_speed();
  /*
   * If speed comes back as 0.0, then we are executing a hard turn, probably as
   * we vector somewhere. In order to have the arrival force work properly, we
   * need to have a velocity with a non-zero length and the correct heading
   * angle at all times. So we report that we have velocity even though we do
   * not, for the purposes of making those calculations work.
   *
   * There probably is a better way to do this, but I don't know what it is. See
   * #585.
   */
  if (speed <= std::numeric_limits<double>::epsilon()) {
    return {0.1, m_sensing->heading()};
  } else {
    return {m_actuation->differential_drive().current_speed(),
            m_sensing->heading()};
  }
} /* linear_velocity() */

double saa_subsystem::angular_velocity(void) const {
  return (m_actuation->differential_drive().right_linspeed() -
          m_actuation->differential_drive().left_linspeed()) /
         m_actuation->differential_drive().axle_length();
} /* angular_velocity() */

double saa_subsystem::max_speed(void) const {
  return m_actuation->differential_drive().max_speed();
} /* max_speed() */

rmath::vector2d saa_subsystem::position(void) const {
  return m_sensing->position();
} /* position() */

NS_END(controller, fordyca);
