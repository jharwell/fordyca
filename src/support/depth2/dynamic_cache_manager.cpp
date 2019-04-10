/**
 * @file dynamic_cache_manager.cpp
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
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/depth2/dynamic_cache_manager.hpp"
#include "fordyca/ds/arena_grid.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/repr/arena_cache.hpp"
#include "fordyca/repr/base_block.hpp"
#include "fordyca/repr/block_cluster.hpp"
#include "fordyca/support/depth2/dynamic_cache_creator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);
using ds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dynamic_cache_manager::dynamic_cache_manager(
    const struct params::caches::caches_params* params,
    ds::arena_grid* const arena_grid)
    : base_cache_manager(arena_grid),
      ER_CLIENT_INIT("fordyca.support.depth2.dynamic_cache_manager"),
      mc_cache_params(*params) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
base_cache_manager::creation_res_t dynamic_cache_manager::create(
    const ds::cache_vector& existing_caches,
    const ds::const_block_cluster_list& clusters,
    ds::block_vector& blocks,
    uint timestep) {
  block_calc_res_t r =
      calc_blocks_for_creation(existing_caches, clusters, blocks);
  if (!r.status) {
    return creation_res_t{false, ds::cache_vector()};
  }

  support::depth2::dynamic_cache_creator::params params = {
      .grid = arena_grid(),
      .cache_dim = mc_cache_params.dimension,
      .min_dist = mc_cache_params.dynamic.min_dist,
      .min_blocks = mc_cache_params.dynamic.min_blocks};
  support::depth2::dynamic_cache_creator creator(&params);

  ds::cache_vector created =
      creator.create_all(existing_caches, r.blocks, timestep);
  caches_created(created.size());

  /*
   * Must be after fixing hidden blocks, otherwise the cache host cell will
   * have a block as its entity!
   */
  creator.update_host_cells(created);
  return creation_res_t{!created.empty(), created};
} /* create() */

base_cache_manager::block_calc_res_t dynamic_cache_manager::calc_blocks_for_creation(
    const ds::cache_vector& existing_caches,
    const ds::const_block_cluster_list& clusters,
    const ds::block_vector& blocks) {
  ds::block_vector to_use;
  std::copy_if(blocks.begin(),
               blocks.end(),
               std::back_inserter(to_use),
               [&](const auto& b) {
                 /* Blocks cannot be in existing caches */
                 return std::all_of(existing_caches.begin(),
                                    existing_caches.end(),
                                    [&](const auto& c) {
                                      return !c->contains_block(b);
                                    }) &&

                        /* blocks cannot be in clusters */
                        std::all_of(clusters.begin(),
                                    clusters.end(),
                                    [&](const auto& clust) {
                                      /* constructed, so must assign before search */
                                      auto cblocks = clust->blocks();
                                      return cblocks.end() ==
                                             std::find(cblocks.begin(),
                                                       cblocks.end(),
                                                       b);
                                    }) &&
                        /* blocks cannot be carried by a robot */
                        -1 == b->robot_id();
               });

  bool ret = true;
  if (to_use.size() < mc_cache_params.dynamic.min_blocks) {
    /*
     * @todo Cannot use std::accumulate for these, because that doesn't work with
     * C++14/gcc7 when you are accumulating into a different type (e.g. from a
     * set of blocks into an int).
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
      accum += "b" + std::to_string(b->id()) + "->fb" +
               std::to_string(b->robot_id()) + ",";
    });
    ER_DEBUG("Block carry statuses: [%s]", accum.c_str());

    accum = "";
    std::for_each(to_use.begin(), to_use.end(), [&](const auto& b) {
      accum += "b" + std::to_string(b->id()) + "->" +
               b->discrete_loc().to_str() + ",";
    });
    ER_DEBUG("Block locations: [%s]", accum.c_str());

    ER_ASSERT(to_use.size() - count < mc_cache_params.dynamic.min_blocks,
              "For new caches, %zu blocks SHOULD be available, but only %zu "
              "are (min=%u)",
              to_use.size() - count,
              to_use.size(),
              mc_cache_params.dynamic.min_blocks);
    ret = false;
  }
  if (to_use.size() < mc_cache_params.static_.size) {
    ER_WARN("Free block count < min blocks for new caches (%zu < %u)",
            to_use.size(),
            mc_cache_params.dynamic.min_blocks);
    ret = false;
  }
  return block_calc_res_t{ret, to_use};
} /* calc_blocks_for_creation() */

NS_END(depth2, support, fordyca);
