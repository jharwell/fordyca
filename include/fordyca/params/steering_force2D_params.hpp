/**
 * @file steering_force2D_params.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_STEERING_FORCE2D_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_STEERING_FORCE2D_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/params/phototaxis_force_params.hpp"
#include "rcppsw/params/base_params.hpp"
#include "rcppsw/robotics/steering2D/force_calculator_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);
namespace steering = rcppsw::robotics::steering2D;

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct steering_force2D_params
 * @ingroup params
 */
struct steering_force2D_params : public steering::force_calculator_params {
  struct phototaxis_force_params phototaxis {};
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_STEERING_FORCE2D_PARAMS_HPP_ */
