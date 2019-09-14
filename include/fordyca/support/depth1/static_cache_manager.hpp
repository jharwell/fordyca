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
#include <boost/optional.hpp>

#include "fordyca/config/caches/caches_config.hpp"
#include "fordyca/support/base_cache_manager.hpp"
#include "fordyca/ds/block_vector.hpp"
#include "fordyca/ds/block_list.hpp"
#include "fordyca/ds/block_cluster_vector.hpp"
#include "fordyca/ds/cache_vector.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/er/client.hpp"
#include "fordyca/support/cache_create_ro_params.hpp"

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
 * cache(s) in the arena.
 */
class static_cache_manager final : public base_cache_manager,
                                   public rer::client<static_cache_manager> {
 public:
  static_cache_manager(const config::caches::caches_config* config,
                       ds::arena_grid* arena_grid,
                       const std::vector<rmath::vector2d>& cache_locs,
                       rmath::rng* rng);
  static_cache_manager(const static_cache_manager&) = delete;
  static_cache_manager& operator=(const static_cache_manager&) = delete;

  /**
   * @brief (Re)-create the static cache in the arena (depth 1 only).
   *
   * @return The created caches. Non-fatal failures to create the static cache
   * can occur if, for example, all blocks are currently being carried by robots
   * and there are not enough free blocks with which to create a cache of the
   * specified minimum size.
   */
  boost::optional<ds::cache_vector> create(const cache_create_ro_params& c_params,
                                           const ds::block_vector&  c_alloc_blocks);

  boost::optional<ds::cache_vector> create_conditional(
      const cache_create_ro_params& c_params,
      const ds::block_vector&  c_alloc_blocks,
      uint n_harvesters,
      uint n_collectors);

  /**
   * @brief Get the # of caches that are being managed.
   */
  size_t n_managed(void) const { return mc_cache_locs.size(); }

 private:
  /**
   * @brief Allocate blocks for static cache(s) re-creation.
   *
   * @param existing_caches The caches that currently exist in the arena.
   * @param blocks Vector of all blocks in the arena.
   *
   * @return A vector of all blocks that will be used in the re-creation of
   * caches in the arena this timestep. There may not be enough free blocks in
   * the arena to meet the desired initial size of at least one cache, which is
   * not an error (all blocks can currently be carried by robots, for example).
   */
  boost::optional<ds::block_vector> blocks_alloc(
      const ds::cache_vector& existing_caches,
      const ds::block_vector& all_blocks) const;

  /**
   * @brief Allocate the blocks that should be used when re-creating cache i.
   *
   * Only blocks that are not:
   *
   * - Already part of an existing cache
   * - Currently carried by a robot
   * - Currently placed on the cell where cache i is to be created
   * - Placed on the cell where any other cache besides cache i *might* be
   *   recreated. We have to allocate blocks so that ALL static caches can be
   *   recreated on the same timestep if needed.
   * - Already allocated for the re-creation of a different static cache
   *
   * are eligible.
   *
   * @param existing_caches Vector of existing static caches.
   * @param allocated_blocks Vector of blocks that have already been allocated
   *                         to the re-creation of other static caches this
   *                         timestep.
   * @param all_blocks All blocks available for cache creation (already
   *                   allocated blocks are not filtered out).
   * @param loc The location the new cache is to be created at.
   * @param n_blocks How many blocks to try to allocate for cache i.
   */
  boost::optional<ds::block_vector> cache_i_blocks_alloc(
      const ds::cache_vector& existing_caches,
      const ds::block_vector& allocated_blocks,
      const ds::block_vector& all_blocks,
      const rmath::vector2d& loc,
      size_t n_blocks) const;

  /**
   * @brief Absorb free blocks that are under caches into the newly created
   * caches.
   *
   * This is necessary for the current static cache creation strategy of
   * randomly picking free blocks in the arena when it is necessary to re-create
   * the cache. There *may* be free blocks that will be within the newly created
   * cache's extent that are NOT considered during cache creation.
   *
   * This is generally only an issue at the start of simulation if random block
   * distribution is used, but weird cases can arise due to task abort+block
   * drop as well, so it is best to be safe and do it unconditionally after
   * creation.
   */
  void post_creation_blocks_absorb(const ds::cache_vector& caches,
                                   const ds::block_vector& blocks);

  /* clang-format off */
  const config::caches::caches_config mc_cache_config;
  const std::vector<rmath::vector2d>  mc_cache_locs;
  rmath::rng*                         m_rng;
  /* clang-format on */
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_STATIC_CACHE_MANAGER_HPP_ */
