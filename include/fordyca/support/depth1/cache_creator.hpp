/**
 * @file cache_creator.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_CACHE_CREATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_CACHE_CREATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <vector>
#include <argos3/core/utility/math/vector2.h>

#include "fordyca/representation/arena_grid.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace representation { class arena_cache; class base_block; }
NS_START(support, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_creator
 * @ingroup support depth1
 *
 * @brief Base class for creating static/dynamic caches in the arena.
 *
 * Used by the arena to actually create caches, and by robots to create
 * "virtual" caches in their on-board representation of the arena.
 */
class cache_creator : public rcppsw::er::client {
 public:
  using cache_vector = std::vector<std::shared_ptr<representation::arena_cache>>;
  using cache_list = std::list<std::shared_ptr<representation::arena_cache>>;
  using block_vector = std::vector<std::shared_ptr<representation::base_block>>;
  using block_list = std::list<std::shared_ptr<representation::base_block>>;

  cache_creator(const std::shared_ptr<rcppsw::er::server>& server,
                representation::arena_grid& grid,
                double cache_size, double resolution);

  /**
   * @brief Create caches from all blocks in the provided list that are close
   * enough together.
   *
   * @return A vector of created caches.
   */
  virtual cache_vector create_all(block_vector& blocks) = 0;

  /**
   * @brief Update the cells for all newly created caches to reflect the fact
   * they now contain caches.
   *
   * @param grid The arena map grid.
   * @param caches Vector of newly created caches.
   */
  static void update_host_cells(representation::arena_grid& grid,
                                cache_vector& caches);

 protected:
  representation::arena_grid& grid(void) const { return m_grid; }
  std::unique_ptr<representation::arena_cache> create_single(
      block_list blocks,
      const argos::CVector2& center);

 private:
  // clang-format off
  double                              m_cache_size;
  double                              m_resolution;
  representation::arena_grid&         m_grid;
  // clang-format on
};
NS_END(support, fordyca, depth1);

#endif // INCLUDE_FORDYCA_SUPPORT_DEPTH1_CACHE_CREATOR_HPP_
