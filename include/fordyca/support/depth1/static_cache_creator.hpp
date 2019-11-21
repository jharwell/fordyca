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
#include <vector>
#include "fordyca/support/base_cache_creator.hpp"
#include "rcppsw/types/spatial_dist.hpp"

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
 * @brief Creates static cache(s) in the arena by taking @ref
 * base_cache::kMinBlocks from the allocated blocks and grouping them together
 * at each of the specified cache locations where a cache does not currently
 * exist in order to create a new static cache.
 */
class static_cache_creator : public base_cache_creator,
                             public rer::client<static_cache_creator> {
 public:
  static_cache_creator(ds::arena_grid* grid,
                       const std::vector<rmath::vector2d>& cache_locs,
                       rtypes::spatial_dist cache_dim);

  /**
   * @brief Re-create all static caches. Ignores block cluster locations because
   * the locations of the static caches do not change and are known to be
   * conflict free.
   */
  ds::cache_vector create_all(const cache_create_ro_params& c_params,
                              const ds::block_vector&  c_alloc_blocks) override;

 private:
  /* clang-format off */
  const std::vector<rmath::vector2d> mc_centers;
  /* clang-format on */
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_STATIC_CACHE_CREATOR_HPP_ */
