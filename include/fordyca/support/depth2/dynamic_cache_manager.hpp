/**
 * @file dynamic_cache_manager.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH2_DYNAMIC_CACHE_MANAGER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH2_DYNAMIC_CACHE_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <utility>
#include <algorithm>
#include <boost/optional.hpp>

#include "fordyca/config/caches/caches_config.hpp"
#include "fordyca/support/base_cache_manager.hpp"
#include "fordyca/ds/block_vector.hpp"
#include "fordyca/ds/cache_vector.hpp"
#include "fordyca/ds/block_cluster_list.hpp"

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
NS_START(support, depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class dynamic_cache_manager
 * @ingroup fordyca support depth2
 *
 * @brief Manager for creation, depletion, and metric gathering for dynamic
 * caches in the arena, whenever they are enabled.
 */
class dynamic_cache_manager final : public base_cache_manager,
                              public rer::client<dynamic_cache_manager> {
 public:
  dynamic_cache_manager(const config::caches::caches_config* config,
                        ds::arena_grid* arena_grid);

  /**
   * @brief Create caches in the arena as needed according to free block
   * configurations.
   *
   * @param existing_caches The list of current caches in the arena.
   * @param blocks The total block vector for the arena.
   * @param clusters The total block clusters in the arena, for use in
   * (possibly) disallowing cache creation within their boundaries, depending on
   * configuration.
   * @param timestep The current timestep.
   *
   * @return \c TRUE iff at least 1 dynamic cache was actually
   * created. Non-fatal failures to create dynamic caches can occur if, for
   * example, all blocks are currently being carried by robots.
   */
  boost::optional<ds::cache_vector> create(const ds::cache_vector& existing_caches,
                                           const ds::const_block_cluster_list& clusters,
                                           ds::block_vector& blocks,
                                           uint timestep);

  /**
   * @brief Get the minimum distance that must be maintained between two caches
   * in order for them to discrete. Equal to the maximum of (twice the cache
   * dimension, minimmum distance between blocks to consider when creating
   * caches);
   */
  double cache_proximity_dist(void) const {
    return std::max(2 * mc_cache_config.dimension,
                    mc_cache_config.dynamic.min_dist);
  }

  /**
   * @brief Get the minimum distance that must be maintained between two blocks
   * in order for them not to be consolidated into a cache. Equal to the minimum
   * cache distance.
   */
  double block_proximity_dist(void) const {
    return mc_cache_config.dynamic.min_dist;
  }

 private:
  /*
   * @brief Calculate the blocks eligible to be considered for dynamic cache
   * creation. Only blocks that are not:
   *
   * - Currently carried by a robot
   * - Currently part of a cache
   *
   * are eligible.
   */
  boost::optional<ds::block_vector> calc_blocks_for_creation(
      const ds::cache_vector& existing_caches,
      const ds::const_block_cluster_list& clusters,
      const ds::block_vector& blocks);

  /* clang-format off */
  const config::caches::caches_config mc_cache_config;
  /* clang-format on */
};

NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_DYNAMIC_CACHE_MANAGER_HPP_ */
