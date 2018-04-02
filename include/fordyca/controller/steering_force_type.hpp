/**
 * @file steering_force_type.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_STEERING_FORCE_TYPE_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_STEERING_FORCE_TYPE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "rcppsw/control/steering_force_type.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * @brief List of steering forces available within the class
 */
class steering_force_type : public rcppsw::control::steering_force_type {
 public:
  enum {
  /**
   * Force pushing robots towards light.
   */
  kPhototaxis = rcppsw::control::steering_force_type::kExternalForces,

  /**
   * Force pushing robots away from light.
   */
  kAntiphototaxis
  };
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_STEERING_FORCE_TYPE_HPP_ */
