/**
 * \file base_cache_creator.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BASE_CACHE_CREATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BASE_CACHE_CREATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>
#include <list>
#include <memory>
#include <utility>
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/spatial_dist.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/ds/arena_grid.hpp"
#include "cosm/foraging/ds/block_vector.hpp"

#include "fordyca/support/cache_create_ro_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_cache_creator
 * \ingroup support
 *
 * \brief Base class for creating static/dynamic caches in the arena.
 */
class base_cache_creator : public rer::client<base_cache_creator> {
 public:
  /**
   * \brief Initialize a new cache creator.
   *
   * \param grid Reference to arena grid.
   * \param cache_dim Dimension of the cache (caches are square so can use a
   *                  scalar).
   */
  base_cache_creator(cds::arena_grid* grid,
                     rtypes::spatial_dist cache_dim);

  base_cache_creator(const base_cache_creator&) = delete;
  base_cache_creator& operator=(const base_cache_creator&) = delete;

  /**
   * \brief (Potentially) create new caches.
   *
   * \param c_params Cache creation parameters
   * \param c_alloc_blocks The blocks that have been allocated for cache
   *                       creation this timestep for all caches.
   *
   * \return A vector of created caches.
   */
  virtual cfds::cache_vector create_all(
      const cache_create_ro_params& c_params,
      const cfds::block_vector2& c_alloc_blocks) = 0;

  /**
   * \brief Update the cells for all newly created caches to reflect the fact
   * they now contain caches.
   *
   * \param caches Vector of newly created caches.
   */
  void update_host_cells(cfds::cache_vector& caches);

  /**
   * \brief Basic sanity checks on newly created caches:
   *
   * - No block contained in one cache is contained in another.
   * - No two newly created caches overlap.
   * - No block that is not currently contained in a cache overlaps any cache.
   * - No cache overlaps a block cluster.
   *
   * \param c_caches The created caches.
   * \param c_free_blocks Blocks that are not carried by a robot or part of a
   *                     newly created cache.
   * \param c_clusters Current block clusters in the arena.
   *
   * \return \c TRUE iff no errors/inconsistencies are found, \c FALSE
   * otherwise.
   */
  bool creation_sanity_checks(
      const cfds::cache_vector& c_caches,
      const cfds::block_vector2& c_free_blocks,
      const cfds::block_cluster_vector& c_clusters) const RCSW_PURE;

 protected:
  const cds::arena_grid* grid(void) const { return m_grid; }
  cds::arena_grid* grid(void) { return m_grid; }

  /**
   * \brief Create a single cache in the arena from the specified set of blocks
   * at the specified location.
   *
   * \param center Location of the new cache.
   * \param blocks Vector of blocks to use to create the cache. Passed by value
   *               because they are (possibly) modified by this function in a
   *               way that callers probably do not want.
   * \param t The current timestep.
   */
  std::unique_ptr<cfrepr::arena_cache> create_single_cache(
      const rmath::vector2d& center,
      cfds::block_vector2 blocks,
      const rtypes::timestep& t);

  rtypes::spatial_dist cache_dim(void) const { return mc_cache_dim; }

 private:
  /* clang-format off */
  const rtypes::spatial_dist mc_cache_dim;

  cds::arena_grid*           m_grid;
  /* clang-format on */
};
NS_END(support, fordyca);

#endif // INCLUDE_FORDYCA_SUPPORT_BASE_CACHE_CREATOR_HPP_
