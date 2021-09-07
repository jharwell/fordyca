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

#ifndef INCLUDE_FORDYCA_SUPPORT_D1_STATIC_CACHE_MANAGER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_D1_STATIC_CACHE_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <boost/optional.hpp>

#include "fordyca/support/base_cache_manager.hpp"
#include "cosm/ds/block3D_vector.hpp"
#include "cosm/foraging/ds/block_cluster_vector.hpp"
#include "cosm/arena/ds/cache_vector.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/er/client.hpp"
#include "fordyca/support/cache_create_ro_params.hpp"
#include "fordyca/ds/block_alloc_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena {
class caching_arena_map;
} /* namespace cosm::arena */

NS_START(fordyca, support, d1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class static_cache_manager
 * \ingroup support d1
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
      const cds::block3D_vectorno& c_all_blocks,
      bool initial);

  boost::optional<cads::acache_vectoro> create_conditional(
      const cache_create_ro_params& c_params,
      const cds::block3D_vectorno& c_all_blocks,
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
   * \param c_usable_blocks Vector of blocks available to use to create caches.
   * \param c_absorbable_blocks Vector of blocks available to absorb into newly
   *                            created caches (i.e., blocks which are in the
   *                            extent of the cache-to-be but which were not
   *                            initially allocated to it for creation. )
   *
   * \return A map of (cache id, block vector) for all caches. There may not be
   * enough free blocks in the arena to meet the desired initial size of at
   * least one cache, which is not an error (all blocks can currently be carried
   * by robots, for example).
   */
  ds::block_alloc_map blocks_alloc(
      const cds::block3D_vectorno& c_usable_blocks,
      const cds::block3D_htno& c_absorbable_blocks) const;

  /**
   * \brief Allocate the blocks that should be used when re-creating cache i.
   *
   * \param c_alloc_map Blocks that have already been allocated to the
   *                    re-creation of other static caches this timestep.
   * \param c_center The location the new cache is to be created at, in real
   *                 coordinates.
   * \param required_blocks How many blocks to try to allocate for cache i.
   */
  boost::optional<cds::block3D_vectorno> cache_i_blocks_alloc(
      const cds::block3D_vectorno& c_usable_blocks,
      const cds::block3D_htno& c_absorbable_blocks,
      const ds::block_alloc_map& c_alloc_map,
      const rmath::vector2d& c_center,
      size_t cache_index,
      size_t required_blocks) const;

  bool cache_i_blocks_alloc_check(const cds::block3D_vectorno& cache_i_blocks,
                                  const rmath::vector2d& c_center) const;

  /*
   * \brief Filter blocks eligible to be considered for cache
   * creation. Only blocks that are not:
   *
   * - Currently carried by a robot
   * - Currently part of a cache
   *
   * are eligible to be USED during cache creation this timestep.
   */
  bool block_alloc_usable_filter(
      const crepr::base_block3D* block,
      const cads::acache_vectorno& existing_caches,
      const cfds::block3D_cluster_vectorro&) const;

  cds::block3D_vectorno cache_i_alloc_from_usable(
      const cds::block3D_vectorno& c_usable_blocks,
      const ds::block_alloc_map& c_alloc_map,
      size_t required_blocks) const;

  /*
   * \brief Calculate the blocks eligible to be considered for absorbtion during
   * cache creation. Absorbable blocks must:
   *
   * - Not be currently carried by a robot
   * - Not currently part of another cache
   *
   * are eligible to be ABSORBED during cache creation this timestep. Blocks in
   * clusters need to be eligible for absorbtion because if a cache-to-be is
   * location on the RIGHT of a cluster (e.g. quad source/powerlaw
   * distribution), then block extents from ramp blocks in the cluster are not
   * considered during the creation process otherwise.
   */
  bool block_alloc_absorbable_filter(
      const crepr::base_block3D* block,
      const cads::acache_vectorno& existing_caches,
      const cfds::block3D_cluster_vectorro&);

  cds::block3D_htno cache_i_alloc_from_absorbable(
      const cds::block3D_htno& c_absorbable_blocks,
      const cds::block3D_vectorno& c_cache_i_blocks,
      const ds::block_alloc_map& c_alloc_map,
      const rmath::vector2d& c_center) const;


  /* clang-format off */
  const std::vector<rmath::vector2d>  mc_cache_locs;
  rmath::rng*                         m_rng;
  /* clang-format on */
};

NS_END(d1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_D1_STATIC_CACHE_MANAGER_HPP_ */
