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
#include <random>
#include <vector>
#include <boost/optional.hpp>

#include "fordyca/config/caches/caches_config.hpp"
#include "fordyca/support/base_cache_manager.hpp"
#include "fordyca/ds/block_vector.hpp"
#include "fordyca/ds/block_list.hpp"
#include "fordyca/ds/block_cluster_vector.hpp"
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
 * cache(s) in the arena, when it is employed.
 */
class static_cache_manager final : public base_cache_manager,
                                   public rer::client<static_cache_manager> {
 public:
  static_cache_manager(const config::caches::caches_config* config,
                       ds::arena_grid* arena_grid,
                       const std::vector<rmath::vector2d>& cache_locs);

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
  boost::optional<ds::cache_vector> create(const ds::cache_vector& existing_caches,
                                           const ds::block_cluster_vector& clusters,
                                           const ds::block_vector& blocks,
                                           uint timestep);

  boost::optional<ds::cache_vector> create_conditional(
      const ds::cache_vector& existing_caches,
      const ds::block_cluster_vector& clusters,
      const ds::block_vector& blocks,
      uint timestep,
      uint n_harvesters,
      uint n_collectors);

  size_t n_managed(void) const { return mc_cache_locs.size(); }

 private:
  /**
   * @brief Compute the blocks that are eligible to be part of the static cache(s)
   * that is/are about to be created.
   *
   * @param blocks Vector of all blocks in the arena.
   *
   * @return \c TRUE iff the calculate of blocks was successful. It may fail if
   * they are not enough free blocks in the arena to meet the desired initial
   * size of the cache (not an error)
   */
  boost::optional<ds::block_vector> calc_blocks_to_use(
      const ds::block_vector& blocks) const;

  boost::optional<ds::block_vector> calc_blocks_for_cache_i(
      const ds::block_vector& all_blocks,
      const ds::block_vector& allocated_blocks,
      const rmath::vector2d& loc) const;

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

  ds::block_list calc_free_blocks(const ds::cache_vector& created_caches,
                                  const ds::block_vector& all_blocks) const;

  /* clang-format off */
  const config::caches::caches_config mc_cache_config;
  const std::vector<rmath::vector2d>  mc_cache_locs;
  std::default_random_engine          m_reng;
  /* clang-format on */
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_STATIC_CACHE_MANAGER_HPP_ */
