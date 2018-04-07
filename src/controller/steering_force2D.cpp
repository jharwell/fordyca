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
#include "fordyca/params/steering_force2D_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
steering_force2D::steering_force2D(
    const std::shared_ptr<rcppsw::er::server>& server,
    steering::boid& entity,
    const params::steering_force2D_params* const params,
    const base_sensing_subsystem& sensors)
    : steering::force_calculator(server, entity, params),
    m_phototaxis_force(&params->phototaxis, sensors) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void steering_force2D::phototaxis(void) {
  argos::CVector2 force = m_phototaxis_force();
  ER_DIAG("Phototaxis force: (%f, %f)", force.GetX(), force.GetY());
  accum_force(force);
} /* phototaxis() */

void steering_force2D::anti_phototaxis(void) {
  argos::CVector2 force = -m_phototaxis_force();
  ER_DIAG("Anti-phototaxis force: (%f, %f)", force.GetX(), force.GetY());
  accum_force(force);
} /* anti_phototaxis() */

NS_END(controller, fordyca);
