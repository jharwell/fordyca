/**
 * @file cache_params.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_DEPTH1_CACHE_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_DEPTH1_CACHE_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/common/base_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth1);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
struct cache_params : public rcppsw::common::base_params {
  bool create_static{false};
  bool create_dynamic{false};
  uint static_size{0};
  double static_respawn_scale_factor{0.0};
  uint usage_penalty{0};
  double dimension{0.0};
  double min_dist{0.0};
};

NS_END(depth1, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_DEPTH1_CACHE_PARAMS_HPP_ */
