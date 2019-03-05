/**
 * @file block_redist_governor_params.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_ARENA_BLOCK_REDIST_GOVERNOR_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_ARENA_BLOCK_REDIST_GOVERNOR_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/params/base_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, arena);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct block_redist_governor_params
 * @ingroup params arena
 */
struct block_redist_governor_params : public rcppsw::params::base_params {
  uint        timestep{0};
  uint        block_count{0};
  std::string trigger{"Null"};
  std::string recurrence_policy{""};
};

NS_END(arena, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_ARENA_BLOCK_REDIST_GOVERNOR_PARAMS_HPP_ */
