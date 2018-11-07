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
#include <argos3/core/utility/math/vector2.h>
#include <list>
#include <vector>

#include "fordyca/ds/arena_grid.hpp"
#include "rcppsw/er/client.hpp"
#include "fordyca/support/cache_vector.hpp"
#include "fordyca/support/block_vector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace representation {
class arena_cache;
class base_block;
} // namespace representation
NS_START(support);
namespace er = rcppsw::er;

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
  using cache_list = std::list<std::shared_ptr<representation::arena_cache>>;
  using block_list = std::list<std::shared_ptr<representation::base_block>>;

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
   *
   * @return A vector of created caches.
   */
  virtual cache_vector create_all(const cache_vector& existing_caches,
                                  block_vector& candidate_blocks) = 0;

  /**
   * @brief Update the cells for all newly created caches to reflect the fact
   * they now contain caches.
   *
   * @param caches Vector of newly created caches.
   */
  void update_host_cells(cache_vector& caches);

 protected:
  const ds::arena_grid* grid(void) const { return m_grid; }
  ds::arena_grid* grid(void) { return m_grid; }
  std::unique_ptr<representation::arena_cache> create_single_cache(
      block_list blocks,
      const argos::CVector2& center);

 private:
  // clang-format off
  double          m_cache_dim;
  ds::arena_grid* m_grid;
  // clang-format on
};
NS_END(fordyca, depth1);

#endif // INCLUDE_FORDYCA_SUPPORT_BASE_CACHE_CREATOR_HPP_
