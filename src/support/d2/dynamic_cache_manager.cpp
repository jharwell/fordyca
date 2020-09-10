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
#include "fordyca/support/d2/dynamic_cache_manager.hpp"

#include "rcppsw/algorithm/transform.hpp"

#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/ds/arena_grid.hpp"
#include "cosm/foraging/repr/block_cluster.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/spatial/dimension_checker.hpp"

#include "fordyca/events/cell2D_empty.hpp"
#include "fordyca/support/d2/dynamic_cache_creator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, d2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dynamic_cache_manager::dynamic_cache_manager(
    const config::caches::caches_config* config,
    carena::caching_arena_map* arena_map,
    rmath::rng* rng)
    : base_cache_manager(arena_map),
      ER_CLIENT_INIT("fordyca.support.d2.dynamic_cache_manager"),
      mc_cache_config(*config),
      m_rng(rng),
      m_map(arena_map) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<cads::acache_vectoro> dynamic_cache_manager::create(
    const cache_create_ro_params& c_params,
    const cds::block3D_vectorno& c_all_blocks) {
  if (auto for_creation = creation_blocks_alloc(c_params.current_caches,
                                                c_params.clusters,
                                                c_all_blocks)) {
    using checker = cspatial::dimension_checker;
    auto even_multiple = checker::even_multiple(arena_map()->grid_resolution(),
                                                mc_cache_config.dimension);
    auto odd_dsize = checker::odd_dsize(arena_map()->grid_resolution(),
                                        even_multiple);

    support::d2::dynamic_cache_creator::params params = {
        .map = m_map,
        .cache_dim = odd_dsize,
        .min_dist = mc_cache_config.dynamic.min_dist,
        .min_blocks = mc_cache_config.dynamic.min_blocks,
        .strict_constraints = mc_cache_config.dynamic.strict_constraints};
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

boost::optional<dynamic_cache_manager::creation_blocks> dynamic_cache_manager::
    creation_blocks_alloc(const cads::acache_vectorno& existing_caches,
                          const cfds::block3D_cluster_vector& clusters,
                          const cds::block3D_vectorno& all_blocks) {
  creation_blocks blocks;

  auto usable_filter = [&](const auto& b) {
    /* Blocks cannot be in existing caches */
    return std::all_of(existing_caches.begin(),
                       existing_caches.end(),
                       [&](const auto& c) { return !c->contains_block(b); }) &&

           /* blocks cannot be in clusters */
           std::all_of(clusters.begin(),
                       clusters.end(),
                       [&](const auto& clust) {
                         /* constructed, so must assign before search */
                         auto cblocks = clust->blocks();
                         ER_DEBUG("cluster blocks: [%s]",
                                 rcppsw::to_string(cblocks).c_str());
                         return cblocks.end() ==
                                std::find(cblocks.begin(), cblocks.end(), b);
                       }) &&
           /* blocks cannot be carried by a robot */
           rtypes::constants::kNoUUID == b->md()->robot_id();
  };

  auto absorbable_filter = [&](const auto& b) {
    /* Blocks cannot be in existing caches */
    return std::all_of(existing_caches.begin(),
                       existing_caches.end(),
                       [&](const auto& c) { return !c->contains_block(b); }) &&

    /* blocks cannot be carried by a robot */
    rtypes::constants::kNoUUID == b->md()->robot_id();
  };
  auto absorbable_transform = [&](auto* b) {
    return std::make_pair(b->id(), b);
  };
  std::copy_if(all_blocks.begin(),
               all_blocks.end(),
               std::back_inserter(blocks.usable),
               usable_filter);
  ralg::transform_if(all_blocks.begin(),
                     all_blocks.end(),
                     std::inserter(blocks.absorbable, blocks.absorbable.begin()),
                     absorbable_filter,
                     absorbable_transform);


  if (creation_blocks_alloc_check(blocks, existing_caches)) {
    return boost::make_optional(blocks);
  } else {
    ER_FATAL_SENTINEL("Bad creation blocks allocation");
    return boost::none;
  }
} /* creation_blocks_alloc() */

bool dynamic_cache_manager::creation_blocks_alloc_check(
    const creation_blocks& c_blocks,
    const cads::acache_vectorno& c_existing_caches) const {

  if (c_blocks.usable.size() < mc_cache_config.dynamic.min_blocks) {
    /*
     * \todo Cannot use std::accumulate for these, because that doesn't work
     * with C++14/gcc7 when you are accumulating into a different type
     * (e.g. from a set of blocks into an int).
     */
    uint count = 0;
    std::for_each(c_blocks.usable.begin(), c_blocks.usable.end(), [&](const auto& b) {
      count +=
          (b->is_out_of_sight() ||
           std::any_of(c_existing_caches.begin(),
                       c_existing_caches.end(),
                       [&](const auto& c) { return !c->contains_block(b); }));
    });

    std::string accum;
    std::for_each(c_blocks.usable.begin(), c_blocks.usable.end(), [&](const auto& b) {
      accum += "b" + rcppsw::to_string(b->id()) + "->fb" +
               rcppsw::to_string(b->md()->robot_id()) + ",";
    });
    ER_DEBUG("Block carry statuses: [%s]", accum.c_str());

    accum = "";
    std::for_each(c_blocks.usable.begin(), c_blocks.usable.end(), [&](const auto& b) {
      accum += "b" +
               rcppsw::to_string(b->id()) + "->" +
               b->ranchor2D().to_str() + "/" +
               b->danchor2D().to_str() +
               ",";
    });
    ER_DEBUG("Block locations: [%s]", accum.c_str());

    ER_CHECK(c_blocks.usable.size() - count < mc_cache_config.dynamic.min_blocks,
             "For new caches, %zu blocks SHOULD be available, but only %zu "
             "are (min=%u)",
             c_blocks.usable.size() - count,
             c_blocks.usable.size(),
             mc_cache_config.dynamic.min_blocks);
  }
  if (c_blocks.usable.size() < mc_cache_config.static_.size) {
    ER_WARN("Free block count < min blocks for new caches (%zu < %u)",
            c_blocks.usable.size(),
            mc_cache_config.dynamic.min_blocks);
  }
  return true;

error:
  return false;
} /* creation_blocks_alloc_check() */

NS_END(d2, support, fordyca);
