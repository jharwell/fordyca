/**
 * @file perceived_arena_map_params.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_DEPTH0_PERCEIVED_ARENA_MAP_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_DEPTH0_PERCEIVED_ARENA_MAP_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/base_params.hpp"
#include "fordyca/params/grid_params.hpp"
#include "fordyca/params/depth0/pheromone_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth0);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct perceived_arena_map_params
 * @ingroup params depth0
 */
struct perceived_arena_map_params : public rcppsw::common::base_params {
  perceived_arena_map_params(void) : grid(), pheromone() {}

  struct grid_params grid;
  struct pheromone_params pheromone;
};

NS_END(depth0, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_DEPTH0_PERCEIVED_ARENA_MAP_PARAMS_HPP_ */
