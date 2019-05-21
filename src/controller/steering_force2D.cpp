/**
 * @file steering_force2D.cpp
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
#include "fordyca/controller/steering_force2D.hpp"
#include "fordyca/config/steering_force2D_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
steering_force2D::steering_force2D(
    steering::boid& entity,
    const config::steering_force2D_config* const config,
    const std::shared_ptr<sensing_subsystem>& sensors)
    : steering::force_calculator(entity, config),
      ER_CLIENT_INIT("fordyca.controller.steering_force2D"),
      m_phototaxis_force(&config->phototaxis, sensors) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void steering_force2D::phototaxis(void) {
  rmath::vector2d force = m_phototaxis_force();
  ER_DEBUG("Phototaxis force: (%f, %f)@%f [%f]",
           force.x(),
           force.y(),
           force.angle().value(),
           force.length());
  accum_force(force);
} /* phototaxis() */

void steering_force2D::anti_phototaxis(void) {
  rmath::vector2d force = -m_phototaxis_force();
  ER_DEBUG("Anti-phototaxis force: (%f, %f)@%f [%f]",
           force.x(),
           force.y(),
           force.angle().value(),
           force.length());
  accum_force(force);
} /* anti_phototaxis() */

NS_END(controller, fordyca);
