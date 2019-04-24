/**
 * @file cache_pickup_params.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
 *
 * This file is part of RCPPSW.
 *
 * RCPPSW is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * RCPPSW is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * RCPPSW.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_FORDYCA_PARAMS_CACHE_SEL_PICKUP_POLICY_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_CACHE_SEL_PICKUP_POLICY_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/params/base_params.hpp"
#include "fordyca/nsalias.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, params, cache_sel);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
struct pickup_policy_params : public rparams::base_params {
  std::string policy{};
  uint        timestep{0};
  uint        cache_count{0};
  uint        cache_size{0};
};

NS_END(cache_sel, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_CACHE_SEL_PICKUP_POLICY_PARAMS_HPP_ */
