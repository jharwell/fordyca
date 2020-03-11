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
#include "fordyca/support/depth2/dynamic_cache_manager.hpp"

#include "cosm/ds/arena_grid.hpp"
#include "cosm/foraging/repr/arena_cache.hpp"
#include "cosm/foraging/repr/block_cluster.hpp"
#include "cosm/repr/base_block2D.hpp"

#include "fordyca/events/cell2D_empty.hpp"
#include "fordyca/support/depth2/dynamic_cache_creator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dynamic_cache_manager::dynamic_cache_manager(
    const config::caches::caches_config* config,
    cds::arena_grid* const arena_grid,
    rmath::rng* rng)
    : base_cache_manager(arena_grid),
      ER_CLIENT_INIT("fordyca.support.depth2.dynamic_cache_manager"),
      mc_cache_config(*config),
      m_rng(rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<cfds::acache_vectoro> dynamic_cache_manager::create(
    const cache_create_ro_params& c_params,
    const cfds::block2D_vectorno& c_alloc_blocks) {
  if (auto to_use = calc_blocks_for_creation(
          c_params.current_caches, c_params.clusters, c_alloc_blocks)) {
    support::depth2::dynamic_cache_creator::params params = {
        .grid = arena_grid(),
        .cache_dim = mc_cache_config.dimension,
        .min_dist = mc_cache_config.dynamic.min_dist,
        .min_blocks = mc_cache_config.dynamic.min_blocks};
    support::depth2::dynamic_cache_creator creator(&params, m_rng);

    cfds::acache_vectoro created = creator.create_all(c_params, *to_use);
    caches_created(created.size());

    /*
     * Must be after fixing hidden blocks, otherwise the cache host cell will
     * have a block as its entity!
     */
    creator.update_host_cells(created);
    return boost::make_optional(created);
  } else {
    return boost::optional<cfds::acache_vectoro>();
  }
} /* create() */

boost::optional<cfds::block2D_vectorno> dynamic_cache_manager::calc_blocks_for_creation(
    const cfds::acache_vectorno& existing_caches,
    const cfds::block_cluster_vector& clusters,
    const cfds::block2D_vectorno& blocks) {
  cfds::block2D_vectorno to_use;
  auto filter = [&](const auto& b) {
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
                         ER_INFO("cluster blocks: [%s]",
                                 rcppsw::to_string(cblocks).c_str());
                         return cblocks.end() ==
                                std::find(cblocks.begin(), cblocks.end(), b);
                       }) &&
           /* blocks cannot be carried by a robot */
    rtypes::constants::kNoUUID == b->md()->robot_id();
  };
  std::copy_if(blocks.begin(), blocks.end(), std::back_inserter(to_use), filter);

  if (to_use.size() < mc_cache_config.dynamic.min_blocks) {
    /*
     * \todo Cannot use std::accumulate for these, because that doesn't work
     * with C++14/gcc7 when you are accumulating into a different type
     * (e.g. from a set of blocks into an int).
     */
    uint count = 0;
    std::for_each(to_use.begin(), to_use.end(), [&](const auto& b) {
      count +=
          (b->is_out_of_sight() ||
           std::any_of(existing_caches.begin(),
                       existing_caches.end(),
                       [&](const auto& c) { return !c->contains_block(b); }));
    });

    std::string accum;
    std::for_each(to_use.begin(), to_use.end(), [&](const auto& b) {
      accum += "b" + rcppsw::to_string(b->id()) + "->fb" +
               rcppsw::to_string(b->md()->robot_id()) + ",";
    });
    ER_DEBUG("Block carry statuses: [%s]", accum.c_str());

    accum = "";
    std::for_each(to_use.begin(), to_use.end(), [&](const auto& b) {
      accum +=
          "b" + rcppsw::to_string(b->id()) + "->" + b->dloc().to_str() + ",";
    });
    ER_DEBUG("Block locations: [%s]", accum.c_str());

    ER_ASSERT(to_use.size() - count < mc_cache_config.dynamic.min_blocks,
              "For new caches, %zu blocks SHOULD be available, but only %zu "
              "are (min=%u)",
              to_use.size() - count,
              to_use.size(),
              mc_cache_config.dynamic.min_blocks);
    return boost::optional<cfds::block2D_vectorno>();
  }
  if (to_use.size() < mc_cache_config.static_.size) {
    ER_WARN("Free block count < min blocks for new caches (%zu < %u)",
            to_use.size(),
            mc_cache_config.dynamic.min_blocks);
    return boost::optional<cfds::block2D_vectorno>();
  }
  return boost::make_optional(to_use);
} /* calc_blocks_for_creation() */

NS_END(depth2, support, fordyca);
