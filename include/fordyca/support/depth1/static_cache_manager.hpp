/**
 * @file static_cache_manager.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_STATIC_CACHE_MANAGER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_STATIC_CACHE_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <utility>
#include <random>

#include "fordyca/params/caches/caches_params.hpp"
#include "fordyca/support/base_cache_manager.hpp"
#include "fordyca/ds/block_vector.hpp"
#include "fordyca/ds/cache_vector.hpp"
#include "rcppsw/math/vector2.hpp"

#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace ds { class arena_grid; }
namespace repr {
class base_block;
class arena_cache;
}
NS_START(support, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class static_cache_manager
 * @ingroup fordyca support depth1
 *
 * @brief Manager for creation, depletion, and metric gathering for the static
 * cache in the arena, when it is employed.
 */
class static_cache_manager : public base_cache_manager,
                             public rer::client<static_cache_manager> {
 public:
  static_cache_manager(const struct params::caches::caches_params* params,
                       ds::arena_grid* arena_grid,
                       const rmath::vector2d& cache_loc);



  /**
   * @brief (Re)-create the static cache in the arena (depth 1 only).
   *
   * @param blocks The total block vector for the arena.
   * @param timestep The current timestep.
   *
   * @return \c TRUE iff a static cache was actually created. Non-fatal failures
   * to create the static cache can occur if, for example, all blocks are
   * currently being carried by robots and there are not enough free blocks with
   * which to create a cache of the specified minimum size.
   */
  creation_res_t create(ds::block_vector& blocks, uint timestep);

  creation_res_t create_conditional(ds::block_vector& blocks,
                                    uint timestep,
                                    uint n_harvesters,
                                    uint n_collectors);

 private:
  /**
   * @brief Compute the blocks that are going to go into the static cache when
   * it is recreated by the arena map.
   *
   * @param grid_resolution The resolution of the arena.
   * @param blocks Empty block vector to be filled with references to the blocks
   *               to be part of the new cache.
   *
   * @return \c TRUE iff the calculate of blocks was successful. It may fail if
   * they are not enough free blocks in the arena to meet the desired initial
   * size of the cache. If it returns \c TRUE, then the second parameter of the
   * pair is the vector of blocks to use for cache creation.
   */
  block_calc_res_t calc_blocks_for_creation(ds::block_vector& blocks);

  /* clang-format off */
  const params::caches::caches_params mc_cache_params;
  const rmath::vector2d               mc_cache_loc;
  std::default_random_engine          m_reng;
  /* clang-format on */
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_STATIC_CACHE_MANAGER_HPP_ */
