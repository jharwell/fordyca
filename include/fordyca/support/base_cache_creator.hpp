/**
 * @file base_cache_creator.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BASE_CACHE_CREATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BASE_CACHE_CREATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <random>
#include <utility>
#include <vector>

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
namespace representation {
class arena_cache;
class base_block;
class multicell_entity;
} // namespace representation
NS_START(support);
namespace er = rcppsw::er;
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class base_cache_creator
 * @ingroup support
 *
 * @brief Base class for creating static/dynamic caches in the arena.
 */
class base_cache_creator : public er::client<base_cache_creator> {
 public:
  /**
   * @brief Initialize a new cache creator.
   *
   * @param grid Reference to arena grid.
   * @param cache_dim Dimension of the cache (caches are square so can use a
   *                  scalar).
   */
  base_cache_creator(ds::arena_grid* grid, double cache_dim);

  base_cache_creator(const base_cache_creator& other) = delete;
  base_cache_creator& operator=(const base_cache_creator& other) = delete;

  /**
   * @brief Create new caches.
   *
   * @param existing_caches Vector of current caches in the arena, for use in
   *                        avoiding overlaps during new cache creation.
   * @param candidate_blocks The vector of free blocks that may be used in cache
   *                         creation.
   * @param cache_dim The dimensions of the caches to create. Possibly needed
   *                  for deconflicting the new cache's location with arena
   *                  boundaries in the cache where there are no existing
   *                  caches that can be used to obtain cache dimensions
   *
   * @return A vector of created caches.
   */
  virtual ds::cache_vector create_all(const ds::cache_vector& existing_caches,
                                      ds::block_vector& candidate_blocks,
                                      double cache_dim) = 0;

  /**
   * @brief Update the cells for all newly created caches to reflect the fact
   * they now contain caches.
   *
   * @param caches Vector of newly created caches.
   */
  void update_host_cells(ds::cache_vector& caches);

 protected:
  struct deconflict_res_t {
    bool status;
    rmath::vector2u loc;
  };

  const ds::arena_grid* grid(void) const { return m_grid; }
  ds::arena_grid* grid(void) { return m_grid; }

  /**
   * @brief Create a single cache in the arena from the specified set of blocks
   * at the specified location. Note that the blocks are passed by value because
   * they are (possibly) modified by this function in a way that callers
   * probably do not want.
   */
  std::unique_ptr<representation::arena_cache> create_single_cache(
      ds::block_list blocks,
      const rmath::vector2d& center);

  /**
   * @brief Basic sanity checks on newly created caches:
   *
   * - No block contained in one cache is contained in another.
   * - No two newly created caches overlap.
   * - No block that is not currently contained in a cache overlaps any cache.
   *
   * This function is provided for derived classes to use when they implement
   * \ref create_all().
   *
   * @return \c TRUE iff no errors/inconsistencies are found, \c FALSE
   * otherwise.
   */
  bool creation_sanity_checks(const ds::cache_vector& new_caches,
                              const ds::block_list& free_blocks) const;

  /**
   * @brief Given the size of the cache-to-be and its tentative location in the
   * arena, return a new location that guarantees that the cache will not
   * overlap arena boundaries.
   *
   * This function is provided for derived classes to use when they implement
   * \ref create_all().
   */
  deconflict_res_t deconflict_loc_boundaries(double cache_dim,
                                             const rmath::vector2u& center) const;

  deconflict_res_t deconflict_loc_entity(
      const representation::multicell_entity* ent,
      const rmath::vector2d& ent_loc,
      const rmath::vector2u& center) const;

 private:
  // clang-format off
  double                             m_cache_dim;
  ds::arena_grid*                    m_grid;
  mutable std::default_random_engine m_rng{};
  // clang-format on
};
NS_END(fordyca, depth1);

#endif // INCLUDE_FORDYCA_SUPPORT_BASE_CACHE_CREATOR_HPP_
