/**
 * \file dynamic_cache_manager.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH2_DYNAMIC_CACHE_MANAGER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH2_DYNAMIC_CACHE_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>
#include <boost/optional.hpp>

#include "fordyca/config/caches/caches_config.hpp"
#include "fordyca/support/base_cache_manager.hpp"
#include "cosm/foraging/ds/block2D_vector.hpp"
#include "cosm/foraging/ds/cache_vector.hpp"
#include "cosm/foraging/ds/block_cluster_vector.hpp"
#include "fordyca/support/cache_create_ro_params.hpp"

#include "rcppsw/math/rng.hpp"

#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dynamic_cache_manager
 * \ingroup support depth2
 *
 * \brief Manager for creation, depletion, and metric gathering for dynamic
 * caches in the arena.
 */
class dynamic_cache_manager final : public base_cache_manager,
                                    public rer::client<dynamic_cache_manager> {
 public:
  dynamic_cache_manager(const config::caches::caches_config* config,
                        cds::arena_grid* arena_grid,
                        rmath::rng* rng);
  dynamic_cache_manager(const dynamic_cache_manager&) = delete;
  dynamic_cache_manager& operator=(const dynamic_cache_manager&) = delete;

  /**
   * \brief Create caches in the arena as needed according to free block
   * configurations.
   *
   * \return The created caches (if any were created).
   */
  boost::optional<cfds::acache_vectoro> create(const cache_create_ro_params& c_params,
                                              const cfds::block2D_vectorno&  c_alloc_blocks);

  /**
   * \brief Get the minimum distance that must be maintained between two caches
   * in order for them to discrete. Equal to the maximum of (twice the cache
   * dimension, minimmum distance between blocks to consider when creating
   * caches);
   */
  rtypes::spatial_dist cache_proximity_dist(void) const {
    return std::max(mc_cache_config.dimension * 2,
                    mc_cache_config.dynamic.min_dist);
  }

 private:
  /*
   * \brief Calculate the blocks eligible to be considered for dynamic cache
   * creation. Only blocks that are not:
   *
   * - Currently carried by a robot
   * - Currently part of a cache
   *
   * are eligible.
   */
  boost::optional<cfds::block2D_vectorno> calc_blocks_for_creation(
      const cfds::acache_vectorno& existing_caches,
      const cfds::block_cluster_vector& clusters,
      const cfds::block2D_vectorno& blocks);

  /* clang-format off */
  const config::caches::caches_config mc_cache_config;
  rmath::rng*                         m_rng;
  /* clang-format on */
};

NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_DYNAMIC_CACHE_MANAGER_HPP_ */
