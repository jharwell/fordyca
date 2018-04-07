/**
 * @file steering_force2D.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_STEERING_FORCE2D_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_STEERING_FORCE2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "rcppsw/common/common.hpp"
#include "fordyca/controller/steering_force_type.hpp"
#include "fordyca/controller/phototaxis_force.hpp"
#include "rcppsw/robotics/steering2D/force_calculator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params { struct steering_force2D_params; }

NS_START(controller);
namespace steering = rcppsw::robotics::steering2D;
class base_sensing_subsystem;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class steering_force2D
 * @ingroup controller
 *
 * @brief Extends \ref rcppsw::robotics::steering2D::force_calculator with
 * steering forces specific to foraging:
 *
 * \ref kPhototaxis
 * \ref kAntiphototaxis
 */
class steering_force2D : public steering::force_calculator {
 public:
  steering_force2D(const std::shared_ptr<rcppsw::er::server>& server,
                   steering::boid& entity,
                   const params::steering_force2D_params* params,
                   const base_sensing_subsystem& sensors);

  /**
   * @brief Add the \ref kPhototaxis force to the sum forces for this timestep.
   */
  void phototaxis(void);

  /**
   * @brief Add the \ref kAntiphototaxis force to the sum forces for this
   * timestep.
   */
  void anti_phototaxis(void);

 private:
  // clang-format off
  phototaxis_force m_phototaxis_force;
  // clang-format on
};

NS_END(control, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_STEERING_FORCE2D_HPP_ */
