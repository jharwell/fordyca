/**
 * @file phototaxis_force.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_PHOTOTAXIS_FORCE_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_PHOTOTAXIS_FORCE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "rcppsw/robotics/steering2D/boid.hpp"

/*******************************************************************************
 * namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace params {
struct phototaxis_force_params;
}

NS_START(controller);
class base_sensing_subsystem;
namespace steering = rcppsw::robotics::steering2D;
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class phototaxis_force
 * @ingroup controller
 *
 * @brief A force pushing the robot away from light sources.
 */
class phototaxis_force {
 public:
  phototaxis_force(const struct params::phototaxis_force_params* params,
                   const std::shared_ptr<base_sensing_subsystem>& sensors);

  rmath::vector2d operator()() const;

  // clang-format off
  double                                   m_max;
  std::shared_ptr<base_sensing_subsystem>  m_sensors;
  // clang-format on
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_PHOTOTAXIS_FORCE_HPP_ */
