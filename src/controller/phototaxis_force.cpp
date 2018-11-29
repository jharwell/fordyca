/**
 * @file phototaxis_force.cpp
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
#include "fordyca/controller/phototaxis_force.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/params/phototaxis_force_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
phototaxis_force::phototaxis_force(
    const struct params::phototaxis_force_params* params,
    const std::shared_ptr<sensing_subsystem>& sensors)
    : m_max(params->max), m_sensors(sensors) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d phototaxis_force::operator()() const {
  rmath::vector2d accum;

  for (auto& r : m_sensors->light().readings()) {
    accum += rmath::vector2d(r.value, rmath::radians(r.angle));
  } /* for(r..) */

  return rmath::vector2d(1.0, accum.angle()) * m_max;
} /* operator()() */

NS_END(controller, fordyca);
