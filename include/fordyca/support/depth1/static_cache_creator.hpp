/**
 * @file static_cache_creator.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_STATIC_CACHE_CREATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_STATIC_CACHE_CREATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/base_cache_creator.hpp"
#include <vector>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class static_cache_creator
 * @ingroup fordyca support depth1
 *
 * @brief Creates a static cache in the arena by taking a specified number of
 * free blocks and grouping them together into a cache at the specified
 * location.
 */
class static_cache_creator : public base_cache_creator,
                             public rer::client<static_cache_creator> {
 public:
  static_cache_creator(ds::arena_grid* grid,
                       const std::vector<rmath::vector2d>& cache_locs,
                       double cache_dim);

  ds::cache_vector create_all(const ds::cache_vector& existing_caches,
                              const ds::block_cluster_vector&,
                              const ds::block_vector& candidate_blocks,
                              uint timestep) override;

 private:
  /* clang-format off */
  const std::vector<rmath::vector2d> mc_centers;
  /* clang-format on */
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_STATIC_CACHE_CREATOR_HPP_ */
