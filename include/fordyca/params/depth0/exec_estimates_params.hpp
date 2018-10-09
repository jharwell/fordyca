/**
 * @file exec_estimates_params.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_DEPTH0_EXEC_ESTIMATES_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_DEPTH0_EXEC_ESTIMATES_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/range.h>
#include "rcppsw/params/base_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth0);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct exec_estimates_params
 * @ingroup params depth0
 *
 * @brief Parameters for execution time estimates of tasks involved in
 * depth0. Not needed for depth0 controllers, but needed in depth > 1
 * controllers to avoid premature abortion upon start due to estimates of 0.0
 * for generalist task.
 */
struct exec_estimates_params : public rcppsw::params::base_params {
  /**
   * @brief Should initial estimates of task execution times be used?
   */
  bool                seed_enabled{false};

  argos::CRange<uint> generalist_range{};
};

NS_END(depth0, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_DEPTH0_EXEC_ESTIMATES_PARAMS_HPP_ */
