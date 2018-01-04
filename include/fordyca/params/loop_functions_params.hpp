/**
 * @file loop_functions_params.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_LOOP_FUNCTIONS_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_LOOP_FUNCTIONS_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <argos3/core/utility/math/range.h>
#include "rcppsw/common/base_params.hpp"
#include "fordyca/params/arena_map_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct loop_functions_params
 * @ingroup params
 */
struct loop_functions_params : public rcppsw::common::base_params {
  loop_functions_params(void) : arena_map() {}

  bool display_robot_id{false};
  bool display_robot_los{false};
  bool display_robot_task{false};
  bool display_block_id{false};
  struct arena_map_params arena_map;
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_LOOP_FUNCTIONS_PARAMS_HPP_ */
