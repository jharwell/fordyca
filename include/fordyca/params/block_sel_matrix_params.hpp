/**
 * @file block_sel_matrix_params.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_BLOCK_SEL_MATRIX_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_BLOCK_SEL_MATRIX_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/params/block_priority_params.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/params/base_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct block_sel_matrix_params
 * @ingroup params
 *
 * @brief XML parameters for the \ref block_sel_matrix
 */
struct block_sel_matrix_params : public rcppsw::params::base_params {
  rmath::vector2d nest{};
  struct block_priority_params priorities {};
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_BLOCK_SEL_MATRIX_PARAMS_HPP_ */
