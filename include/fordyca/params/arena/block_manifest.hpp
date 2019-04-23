/**
 * @file block_manifest.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_ARENA_BLOCK_MANIFEST_HPP_
#define INCLUDE_FORDYCA_PARAMS_ARENA_BLOCK_MANIFEST_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/params/base_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, arena);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct block_manifest
 * @ingroup fordyca params arena
 *
 * @brief Params of what types of blocks and how many of each should be
 * created, as well as size and other characteristics.
 */
struct block_manifest : public rcppsw::params::base_params {
  uint n_cube{0};  /// # cube blocks to distribute in arena
  uint n_ramp{0};  /// # ramp blocks to distribute in arena

  /**
   * @brief Size in meters of the unit dimension for blocks. Cube blocks are 1x1
   * in this dimension, and ramp blocks are 2x1.
   */
  double unit_dim{0.0};
};

NS_END(arena, params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_ARENA_BLOCK_MANIFEST_HPP_ */
