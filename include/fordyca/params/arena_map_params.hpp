/**
 * @file arena_map_params.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_ARENA_MAP_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_ARENA_MAP_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>
#include "fordyca/params/block_params.hpp"
#include "fordyca/params/depth1/cache_params.hpp"
#include "fordyca/params/grid_params.hpp"
#include "rcppsw/params/base_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct arena_map_params
 * @ingroup params
 */
struct arena_map_params : public rcppsw::params::base_params {
  struct grid_params grid{};
  struct block_params block{};
  struct depth1::cache_params cache{};

  argos::CVector2 nest_center{};
  argos::CRange<double> nest_x{};
  argos::CRange<double> nest_y{};
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_ARENA_MAP_PARAMS_HPP_ */
