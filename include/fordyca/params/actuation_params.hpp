/**
 * @file actuation_params.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_ACTUATION_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_ACTUATION_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/nsalias.hpp"
#include "fordyca/params/steering_force2D_params.hpp"
#include "rcppsw/params/base_params.hpp"
#include "rcppsw/robotics/kinematics2D/differential_drive_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);
namespace kinematics2D = rcppsw::robotics::kinematics2D;

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/

/**
 * @struct actuation_params
 * @ingroup fordyca params
 */
struct actuation_params : public rparams::base_params {
  struct kinematics2D::differential_drive_params differential_drive {};
  struct steering_force2D_params steering {};
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_ACTUATION_PARAMS_HPP_ */
