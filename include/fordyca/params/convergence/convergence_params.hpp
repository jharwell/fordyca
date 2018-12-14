/**
 * @file convergence_params.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_CONVERGENCE_CONVERGENCE_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_CONVERGENCE_CONVERGENCE_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/params/base_params.hpp"
#include "fordyca/params/convergence/positional_entropy_params.hpp"
#include "fordyca/params/convergence/interactivity_params.hpp"
#include "fordyca/params/convergence/angular_order_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, convergence);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct convergence_params
 * @ingroup params
 */
struct convergence_params : public rcppsw::params::base_params {
  struct positional_entropy_params pos_entropy{};
  struct interactivity_params      interactivity{};
  struct angular_order_params      ang_order{};
};

NS_END(convergence, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_CONVERGENCE_CONVERGENCE_PARAMS_HPP_ */
