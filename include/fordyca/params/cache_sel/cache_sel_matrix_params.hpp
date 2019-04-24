/**
 * @file cache_sel_matrix_params.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_CACHE_SEL_CACHE_SEL_MATRIX_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_CACHE_SEL_CACHE_SEL_MATRIX_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/math/range.hpp"
#include "rcppsw/params/base_params.hpp"
#include "fordyca/params/cache_sel/pickup_policy_params.hpp"
#include "fordyca/nsalias.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, cache_sel);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct cache_sel_matrix_params
 * @ingroup fordyca params cache_sel
 *
 * @brief XML parameters for the \ref cache_sel_matrix
 */
struct cache_sel_matrix_params : public rparams::base_params {
  double cache_prox_dist{0.0};
  double block_prox_dist{0.0};
  double nest_prox_dist{0.0};
  double cluster_prox_dist{0.0};
  rcppsw::math::rangeu site_xrange{0, 0};
  rcppsw::math::rangeu site_yrange{0, 0};
  pickup_policy_params initial_pickup{};
};

NS_END(cache_sel, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_CACHE_SEL_CACHE_SEL_MATRIX_PARAMS_HPP_ */
