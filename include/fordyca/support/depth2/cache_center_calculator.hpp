/**
 * @file cache_center_calculator.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH2_CACHE_CENTER_CALCULATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH2_CACHE_CENTER_CALCULATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <random>
#include <utility>
#include <vector>
#include <boost/optional.hpp>

#include "fordyca/ds/arena_grid.hpp"
#include "fordyca/ds/block_list.hpp"
#include "fordyca/ds/block_vector.hpp"
#include "fordyca/ds/cache_vector.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace repr {
class arena_cache;
class base_block;
class multicell_entity;
} // namespace repr
NS_START(support, depth2);
namespace er = rcppsw::er;
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_center_calculator
 * @ingroup support
 *
 * @brief Calculate the center for a cache to create in the arena:
 *
 * Given:
 * - The blocks that should be included the cache (initial guess for center is
 *   the average x/y coordinates of elements in this list).
 * - The block that should not be included in the cache and need to be avoided
 *   during placement.
 * - Existing caches that need to be avoided during placement.
 */
class cache_center_calculator : public er::client<cache_center_calculator> {
 public:
  static constexpr uint kOVERLAP_SEARCH_MAX_TRIES = 10;

  /**
   * @brief Initialize a new cache calculator.
   *
   * @param grid Reference to arena grid.
   * @param cache_dim Dimension of the cache (caches are square so can use a
   *                  scalar).
   */
  cache_center_calculator(ds::arena_grid* grid, double cache_dim);

  cache_center_calculator(const cache_center_calculator& other) = delete;
  cache_center_calculator& operator=(const cache_center_calculator& other) = delete;

  /**
   * @brief Calculate the center of the new cache that will be constructed from
   * the specified blocks.
   *
   * Ideally that will be just the average of the x and y coordinates of all the
   * constituent blocks. However, it is possible that placing a cache at that
   * location will cause it to overlap with other caches, and so corrections may
   * be necessary. We also need to deconflict the new cache location from
   * existing blocks in the arena, as it is possible that blocks that are too
   * far away to be considered part of our new cache will overlap it if it moves
   * around to deconflict with existing caches.
   *
   * @param candidate_blocks The list of blocks to create a new cache from.
   * @param existing_caches Vector of existing caches in the arena.
   * @param nc_blocks List of free (non-candidate) blocks in the arena that are
   *                  NOT going to be part of the new cache.
   *
   * @return Coordinates of the new cache, if any were found.
   */
  boost::optional<rmath::vector2u> operator()(
      const ds::block_list& cache_i_blocks,
      const ds::block_list& nc_blocks,
      const ds::cache_vector& existing_caches) const;

 private:
  /**
   * @brief Deconflict the guessed cache center from overlap with existing
   * caches and blocks that are not going to be part of the cache.
   *
   * @return An updated cache center, if one is needed.
   */
  boost::optional<rmath::vector2u> deconflict_loc(
      const ds::block_list& nc_blocks,
      const ds::cache_vector& existing_caches,
      const rmath::vector2u& center) const;

  /**
   * @brief Given the size of the cache-to-be and its tentative location in the
   * arena, possibily return a new location that does not overlap arena
   * boundaries if the current one does.
   *
   * This function is provided for derived classes to use when they implement
   * \ref create_all().
   *
   * @param center The tentative location of the cache. It is an integer
   *               location, but it is a *REAL* location (i.e. not
   *               discretized).
   */
  boost::optional<rmath::vector2u> deconflict_loc_boundaries(
      const rmath::vector2u& center) const;

  /**
   * @brief Given an existing entity (cache or a block) that must be avoided
   * during placement, the tentatitive location of the cache-to-be and its
   * dimensions, possibly return a new cache center if the current location
   * causes an overlap.
   *
   * @param center The tentative location of the cache. It is an integer
   *               location, but it is a *REAL* location (i.e. not
   *               discretized).
   */
  boost::optional<rmath::vector2u> deconflict_loc_entity(
      const repr::multicell_entity* ent,
      const rmath::vector2d& ent_loc,
      const rmath::vector2u& center) const;

 private:
  /* clang-format off */
  double                             m_cache_dim;
  ds::arena_grid*                    m_grid;
  mutable std::default_random_engine m_rng;
  /* clang-format on */
};
NS_END(depth2, support, fordyca);

#endif // INCLUDE_FORDYCA_SUPPORT_DEPTH2_CACHE_CENTER_CALCULATOR_HPP_