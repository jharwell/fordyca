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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>

#include "rcppsw/math/rng.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/ds/block3D_vector.hpp"
#include "cosm/arena/ds/cache_vector.hpp"
#include "cosm/foraging/ds/block_cluster_vector.hpp"

#include "fordyca/argos/support/caches/create_ro_params.hpp"
#include "fordyca/argos/support/caches/config/caches_config.hpp"
#include "fordyca/argos/support/caches/base_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena {
class caching_arena_map;
} /* namespace cosm::arena */

NS_START(fordyca, argos, support, d2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dynamic_cache_manager
 * \ingroup argos support d2
 *
 * \brief Manager for creation, depletion, and metric gathering for dynamic
 * caches in the arena.
 */
class dynamic_cache_manager final : public fascaches::base_manager,
                                    public rer::client<dynamic_cache_manager> {
 public:
  dynamic_cache_manager(const fascaches::config::caches_config* config,
                        carena::caching_arena_map* arena_map,
                        rmath::rng* rng);
  dynamic_cache_manager(const dynamic_cache_manager&) = delete;
  dynamic_cache_manager& operator=(const dynamic_cache_manager&) = delete;

  /**
   * \brief Create caches in the arena as needed according to free block
   * configurations.
   *
   * \return The created caches (if any were created).
   */
  boost::optional<cads::acache_vectoro> create(const fascaches::create_ro_params& c_params,
                                              const cds::block3D_vectorno&  c_all_blocks);

  /**
   * \brief Get the minimum distance that must be maintained between two caches
   * in order for them to discrete. Equal to the maximum of (twice the cache
   * dimension, minimmum distance between blocks to consider when creating
   * caches);
   */
  rspatial::euclidean_dist cache_proximity_dist(void) const {
    return std::max(config()->dimension * 2, config()->dynamic.min_dist);
  }

 private:
  /*
   * \brief Filter blocks eligible to be considered for cache
   * creation. Only blocks that are not:
   *
   * - Currently carried by a robot
   * - Currently part of a cache
   * - Currently part of a block cluster.
   *
   * are eligible to be USED during cache creation this timestep.
   */
  bool block_alloc_usable_filter(
      const crepr::sim_block3D* block,
      const cads::acache_vectorno& existing_caches,
      const cfds::block3D_cluster_vectorro& clusters);

  /*
   * \brief Calculate the blocks eligible to be considered for absorbtion during
   * cache creation. Only blocks that are not:
   *
   * - Currently carried by a robot
   * - Currently part of a cache
   *
   * are eligible to be ABSORBED during cache creation this timestep. Blocks in
   * clusters need to be eligible for absorbtion because if a cache-to-be is
   * location on the RIGHT of a cluster (e.g. quad source/powerlaw
   * distribution), then block extents from ramp blocks in the cluster are not
   * considered during the creation process otherwise.
   */
  bool block_alloc_absorbable_filter(
      const crepr::sim_block3D* block,
      const cads::acache_vectorno& existing_caches,
      const cfds::block3D_cluster_vectorro&);

  /* clang-format off */
  rmath::rng*                         m_rng;
  carena::caching_arena_map*          m_map;
  /* clang-format on */
};

NS_END(d2, support, argos, fordyca);

