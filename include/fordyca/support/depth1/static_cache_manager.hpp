/**
 * \file static_cache_manager.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "cosm/ds/block3D_vector.hpp"
#include "cosm/foraging/ds/block_cluster_vector.hpp"
#include "cosm/arena/ds/cache_vector.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/er/client.hpp"
#include "fordyca/support/cache_create_ro_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena {
class caching_arena_map;
} /* namespace cosm::arena */

NS_START(fordyca, support, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class static_cache_manager
 * \ingroup support depth1
 *
 * \brief Manager for creation, depletion, and metric gathering for the static
 * cache(s) in the arena.
 */
class static_cache_manager final : public base_cache_manager,
                                   public rer::client<static_cache_manager> {
 public:
  static_cache_manager(const config::caches::caches_config* config,
                       carena::caching_arena_map* map,
                       const std::vector<rmath::vector2d>& cache_locs,
                       rmath::rng* rng);
  static_cache_manager(const static_cache_manager&) = delete;
  static_cache_manager& operator=(const static_cache_manager&) = delete;

  /**
   * \brief (Re)-create the static cache(s) in the arena.
   *
   * \return The created caches. Non-fatal failures to create the static cache
   * can occur if, for example, all blocks are currently being carried by robots
   * and there are not enough free blocks with which to create a cache of the
   * specified minimum size.
   *
   */
  boost::optional<cads::acache_vectoro> create(
      const cache_create_ro_params& c_params,
      const cds::block3D_vectorno&  c_alloc_blocks,
      bool initial);

  boost::optional<cads::acache_vectoro> create_conditional(
      const cache_create_ro_params& c_params,
      const cds::block3D_vectorno&  c_alloc_blocks,
      size_t n_harvesters,
      size_t n_collectors);

  /**
   * \brief Get the # of caches that are being managed.
   */
  size_t n_managed(void) const { return mc_cache_locs.size(); }

 private:
  /**
   * \brief Allocate blocks for static cache(s) re-creation.
   *
   * \param existing_caches The caches that currently exist in the arena.
   * \param blocks Vector of all blocks in the arena.
   *
   * \return A vector of all blocks that will be used in the re-creation of
   * caches in the arena this timestep. There may not be enough free blocks in
   * the arena to meet the desired initial size of at least one cache, which is
   * not an error (all blocks can currently be carried by robots, for example).
   */
  boost::optional<cds::block3D_vectorno> blocks_alloc(
      const cads::acache_vectorno& existing_caches,
      const cds::block3D_vectorno& all_blocks) const;

  /**
   * \brief Allocate the blocks that should be used when re-creating cache i.
   *
   * Only blocks that are not:
   *
   * - Already part of an existing cache
   * - Currently carried by a robot
   * - Currently placed on the cell where cache i is to be created
   * - Placed on the cell where any other cache besides cache i *might* be
   *   recreated. We have to allocate blocks so that ALL static caches can be
   *   recreated on the same timestep if needed
   * - Already allocated for the re-creation of a different static cache
   *
   * are eligible.
   *
   * \param existing_caches Vector of existing static caches.
   * \param allocated_blocks Vector of blocks that have already been allocated
   *                         to the re-creation of other static caches this
   *                         timestep.
   * \param all_blocks All blocks available for cache creation (already
   *                   allocated blocks are not filtered out).
   * \param center The location the new cache is to be created at.
   * \param n_blocks How many blocks to try to allocate for cache i.
   */
  boost::optional<cds::block3D_vectorno> cache_i_blocks_alloc(
      const cads::acache_vectorno& existing_caches,
      const cds::block3D_vectorno& allocated_blocks,
      const cds::block3D_vectorno& all_blocks,
      const rmath::vector2z& center,
      size_t n_blocks) const;

  /**
   * \brief Absorb free blocks that are under caches into the newly created
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
  void post_creation_blocks_absorb(const cads::acache_vectoro& caches,
                                   const cds::block3D_vectorno& blocks);

  /* clang-format off */
  const config::caches::caches_config mc_cache_config;
  const std::vector<rmath::vector2d>  mc_cache_locs;
  rmath::rng*                         m_rng;
  /* clang-format on */
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_STATIC_CACHE_MANAGER_HPP_ */
