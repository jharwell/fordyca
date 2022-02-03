/**
 * \file dynamic_cache_manager.cpp
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
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/argos/support/d2/dynamic_cache_manager.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/foraging/repr/block_cluster.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/spatial/dimension_checker.hpp"

#include "fordyca/events/cell2D_empty.hpp"
#include "fordyca/argos/support/d2/dynamic_cache_creator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, d2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dynamic_cache_manager::dynamic_cache_manager(
    const fascaches::config::caches_config* config,
    carena::caching_arena_map* arena_map,
    rmath::rng* rng)
    : base_manager(config, arena_map),
      ER_CLIENT_INIT("fordyca.support.d2.dynamic_cache_manager"),
      m_rng(rng),
      m_map(arena_map) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<cads::acache_vectoro>
dynamic_cache_manager::create(const fascaches::create_ro_params& c_params,
                              const cds::block3D_vectorno& c_all_blocks) {
  auto usable_cb = std::bind(&dynamic_cache_manager::block_alloc_usable_filter,
                                 this,
                                 std::placeholders::_1,
                                 std::placeholders::_2,
                                 std::placeholders::_3);
  auto absorbable_cb = std::bind(&dynamic_cache_manager::block_alloc_absorbable_filter,
                                 this,
                                 std::placeholders::_1,
                                 std::placeholders::_2,
                                 std::placeholders::_3);
  if (auto for_creation = creation_blocks_alloc(c_all_blocks,
                                                c_params.current_caches,
                                                c_params.clusters,
                                                usable_cb,
                                                absorbable_cb)) {
    using checker = cspatial::dimension_checker;
    auto even_multiple = checker::even_multiple(arena_map()->grid_resolution(),
                                                config()->dimension);
    auto odd_dsize =
        checker::odd_dsize(arena_map()->grid_resolution(), even_multiple);

    support::d2::dynamic_cache_creator::params params = {
      .map = m_map,
      .cache_dim = odd_dsize,
      .min_dist = config()->dynamic.min_dist,
      .min_blocks = config()->dynamic.min_blocks,
      .strict_constraints = config()->strict_constraints
    };
    support::d2::dynamic_cache_creator creator(&params, m_rng);

    auto res = creator.create_all(c_params,
                                  std::move(for_creation->usable),
                                  std::move(for_creation->absorbable));
    caches_created(res.created.size());
    caches_discarded(res.n_discarded);

    /* Configure cache extents */
    creator.cache_extents_configure(res.created);

    /* update bloctree */
    bloctree_update(res.created);

    return boost::make_optional(res.created);
  } else {
    return boost::optional<cads::acache_vectoro>();
  }
} /* create() */

bool dynamic_cache_manager::block_alloc_usable_filter(
    const crepr::base_block3D* block,
    const cads::acache_vectorno& existing_caches,
    const cfds::block3D_cluster_vectorro& clusters) {
  /*
   * Initial allocation.
   *
   * Note that the calculations for membership are ordered from least to most
   * computationally expensive to compute, so don't reorder them willy-nilly.
   */
  return
      /* blocks cannot be carried by a robot */
      !block->is_carried_by_robot() &&

      /* blocks cannot be in existing caches */
      std::all_of(existing_caches.begin(),
                     existing_caches.end(),
                     [&](const auto& c) { return !c->contains_block(block); }) &&

      /* blocks cannot be in clusters */
      std::all_of(clusters.begin(),
                  clusters.end(),
                  [&](const auto& clust) {
                    /* constructed, so must assign before search */
                    auto cblocks = clust->blocks();
                    ER_DEBUG("Cluster%d blocks: [%s]",
                             clust->id().v(),
                             rcppsw::to_string(cblocks).c_str());
                    return cblocks.end() == std::find(cblocks.begin(),
                                                      cblocks.end(),
                                                      block);
                  });
} /* block_alloc_usable_filter() */

bool dynamic_cache_manager::block_alloc_absorbable_filter(
    const crepr::base_block3D* block,
    const cads::acache_vectorno& existing_caches,
    const cfds::block3D_cluster_vectorro&) {
  /* blocks cannot be carried by a robot */
  return !block->is_carried_by_robot() &&
      /* Blocks cannot be in existing caches */
       std::all_of(existing_caches.begin(),
                   existing_caches.end(),
                   [&](const auto& c) { return !c->contains_block(block); });
} /* block_alloc_absorbable_filter() */

NS_END(d2, support, argos, fordyca);
