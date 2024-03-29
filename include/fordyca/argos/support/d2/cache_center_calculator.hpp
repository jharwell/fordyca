/**
 * \file cache_center_calculator.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <utility>
#include <vector>
#include <boost/optional.hpp>

#include "cosm/arena/ds/arena_grid.hpp"
#include "cosm/ds/block3D_vector.hpp"
#include "cosm/arena/ds/cache_vector.hpp"
#include "cosm/foraging/ds/block_cluster_vector.hpp"
#include "cosm/arena/ds/nest_vector.hpp"

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/spatial_dist.hpp"
#include "rcppsw/math/rng.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class entity2D;
} /* namespace cosm::repr */

NS_START(fordyca, argos, support, d2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_center_calculator
 * \ingroup argos support d2
 *
 * \brief Calculate the center for a cache to create in the arena:
 *
 * Given:
 * - The blocks that should be included the cache (initial guess for center is
 *   the average x/y coordinates of elements in this list).
 * - Existing caches that need to be avoided during placement.
 */
class cache_center_calculator : public rer::client<cache_center_calculator> {
 public:
  static constexpr size_t kOVERLAP_SEARCH_MAX_TRIES = 100;

  /**
   * \brief Initialize a new cache calculator.
   *
   * \param grid Reference to arena grid.
   * \param cache_dim Dimension of the cache (caches are square so can use a
   *                  scalar).
   * \param nests The nests in the arena.
   * \param c_clusters Vector of block clusters in the area.
   */
  cache_center_calculator(cads::arena_grid* grid,
                          const rtypes::spatial_dist& cache_dim,
                          const cads::nest_vectorro& c_nests,
                          const cfds::block3D_cluster_vectorro& c_clusters);

  cache_center_calculator(const cache_center_calculator&) = delete;
  cache_center_calculator& operator=(const cache_center_calculator&) = delete;

  /**
   * \brief Calculate the center of the new cache that will be constructed from
   * the specified blocks.
   *
   * Ideally that will be just the average of the x and y coordinates of all the
   * constituent blocks. However, it is possible that placing a cache at that
   * location will cause it to overlap with other caches, and so corrections may
   * be necessary.
   *
   * \param c_cache_i_blocks The list of blocks to create a new cache from.
   * \param c_existing_caches Vector of existing caches in the arena.
   * \param rng RNG to use duration cache center calculation (guess and check if
   *            initial computed location does not work).
   *
   * \return Coordinates of the new cache, if any were found.
   */
  boost::optional<rmath::vector2d> operator()(
      const cds::block3D_vectorno& c_cache_i_blocks,
      const cads::acache_vectorno& c_existing_caches,
      rmath::rng* rng) const;

 private:
  /**
   * \brief Deconflict the guessed cache center from overlap with existing
   * caches and blocks that are not going to be part of the cache.
   *
   * \return An updated cache center, if one is needed.
   */
  boost::optional<rmath::vector2d> deconflict_loc(
      const cads::acache_vectorno& c_existing_caches,
      const rmath::vector2d& c_center,
      rmath::rng* rng) const;

  /**
   * \brief Given the size of the cache-to-be and its tentative location in the
   * arena, possibily return a new location that does not overlap arena
   * boundaries if the current one does.
   *
   * \param center The tentative location of the cache. It is an integer
   *               location, but it is a *REAL* location (i.e. not
   *               discretized).
   */
  boost::optional<rmath::vector2d> deconflict_loc_boundaries(
      const rmath::vector2d& c_center) const;

  /**
   * \brief Given an existing entity (cache or a block) that must be avoided
   * during placement, the tentatitive location of the cache-to-be and its
   * dimensions, possibly return a new cache center if the current location
   * causes an overlap.
   *
   * \param center The tentative location of the cache. It is an integer
   *               location, but it is a *REAL* location (i.e. not
   *               discretized).
   */
  boost::optional<rmath::vector2d> deconflict_loc_entity(
      const crepr::entity2D* ent,
      const rmath::vector2d& c_center,
      rmath::rng* rng) const;

  /* clang-format off */
  const rtypes::spatial_dist         mc_cache_dim;
  const cads::nest_vectorro          mc_nests;
  const cfds::block3D_cluster_vectorro mc_clusters;

  cads::arena_grid*                  m_grid;
  /* clang-format on */
};
NS_END(d2, support, argos, fordyca);

