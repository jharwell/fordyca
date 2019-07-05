/**
 * @file static_cache_manager.cpp
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
#include "fordyca/support/depth1/static_cache_manager.hpp"
#include <chrono>

#include "fordyca/ds/arena_grid.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/math/cache_respawn_probability.hpp"
#include "fordyca/repr/arena_cache.hpp"
#include "fordyca/repr/base_block.hpp"
#include "fordyca/support/depth1/static_cache_creator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);
using ds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
static_cache_manager::static_cache_manager(
    const config::caches::caches_config* config,
    ds::arena_grid* const arena_grid,
    const std::vector<rmath::vector2d>& cache_locs)
    : base_cache_manager(arena_grid),
      ER_CLIENT_INIT("fordyca.support.depth1.static_cache_manager"),
      mc_cache_config(*config),
      mc_cache_locs(cache_locs),
      m_reng(std::chrono::system_clock::now().time_since_epoch().count()) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<ds::cache_vector> static_cache_manager::create(
    const ds::cache_vector& existing_caches,
    const ds::block_cluster_vector& clusters,
    const ds::block_vector& blocks,
    uint timestep) {
  ER_DEBUG("(Re)-Creating static cache(s)");
  ER_ASSERT(mc_cache_config.static_.size >= repr::base_cache::kMinBlocks,
            "Static cache size %u < minimum %zu",
            mc_cache_config.static_.size,
            repr::base_cache::kMinBlocks);

  auto to_use = calc_blocks_to_use(blocks);
  if (!to_use) {
    ER_WARN("Unable to create static cache(s): Not enough free blocks");
    return boost::optional<ds::cache_vector>();
  }
  static_cache_creator creator(arena_grid(),
                               mc_cache_locs,
                               mc_cache_config.dimension);

  auto created = creator.create_all(
      existing_caches, ds::block_cluster_vector(), *to_use, timestep);

  /*
   * Fix hidden blocks and update host cells. Host cell updating must be second,
   * otherwise the cache host cell will have a block as its entity!
   */
  post_creation_blocks_absorb(created, blocks);
  creator.update_host_cells(created);

  auto free_blocks = calc_free_blocks(created, blocks);
  ER_ASSERT(creator.creation_sanity_checks(created, free_blocks, clusters),
            "One or more bad caches on creation");

  caches_created(created.size());
  return boost::make_optional(created);
} /* create() */

boost::optional<ds::cache_vector> static_cache_manager::create_conditional(
    const ds::cache_vector& existing_caches,
    const ds::block_cluster_vector& clusters,
    const ds::block_vector& blocks,
    uint timestep,
    uint n_harvesters,
    uint n_collectors) {
  math::cache_respawn_probability p(
      mc_cache_config.static_.respawn_scale_factor);
  std::uniform_real_distribution<> dist(0.0, 1.0);

  if (p.calc(n_harvesters, n_collectors) >= dist(m_reng)) {
    return create(existing_caches, clusters, blocks, timestep);
  } else {
    return boost::optional<ds::cache_vector>();
  }
} /* create_conditional() */

ds::block_list static_cache_manager::calc_free_blocks(
    const ds::cache_vector& created_caches,
    const ds::block_vector& all_blocks) const {
  ds::block_list free_blocks;
  std::copy_if(all_blocks.begin(),
               all_blocks.end(),
               std::back_inserter(free_blocks),
               [&](const auto& b) __rcsw_pure {
                 /* block not carried by robot */
                 return -1 == b->robot_id() &&
                     /* block not used in cache creation */
                     all_blocks.end() == std::find(all_blocks.begin(),
                                                   all_blocks.end(), b) &&
                     /*
                      * Block not inside cache (to catch blocks that were on the
                      * host cell for the cache, and we incorporated into it
                      * during creation)
                      */
                     std::all_of(created_caches.begin(),
                                 created_caches.end(),
                                 [&](const auto& c) {
                                   return !c->contains_block(b);
                                  });
                     });
  return free_blocks;
} /* calc_free_blocks() */

boost::optional<ds::block_vector> static_cache_manager::calc_blocks_to_use(
    const ds::block_vector& blocks) const {

  ds::block_vector to_use;
  for (auto &loc : mc_cache_locs) {
    if (auto cache_i = calc_blocks_for_cache_i(blocks, to_use, loc)) {
      to_use.insert(to_use.end(),
                    cache_i->begin(),
                    cache_i->end());
    }
  } /* for(&loc..) */

  if (to_use.empty()) {
    return boost::optional<ds::block_vector>();
  } else {
    return boost::make_optional(to_use);
  }
} /* calc_blocks_to_use() */

boost::optional<ds::block_vector> static_cache_manager::calc_blocks_for_cache_i(
    const ds::block_vector& all_blocks,
    const ds::block_vector& allocated_blocks,
    const rmath::vector2d& loc) const {

  /*
   * Only blocks that are not:
   *
   * - Currently carried by a robot
   * - Currently placed on the cell where the cache is to be created
   * - Already allocated for the re-creation of a different static cache
   *
   * are eligible for being used to re-create a the current static cache.
   */

  ds::block_vector cache_i_blocks;
  rmath::vector2u dcenter = rmath::dvec2uvec(loc,
                                             arena_grid()->resolution());
    std::copy_if(all_blocks.begin(),
                 all_blocks.end(),
                 std::back_inserter(cache_i_blocks),
                 [&](const auto& b) {
                   /* not carried by robot */
                   return -1 == b->robot_id() &&
                       /* not on the cell where the cache is to be created */
                       b->dloc() != dcenter &&
                       /* not already allocated for a different cache */
                       allocated_blocks.end() == std::find(allocated_blocks.begin(),
                                                 allocated_blocks.end(),
                                                 b);
                 });

  if (cache_i_blocks.size() < repr::base_cache::kMinBlocks) {
    /*
     * Cannot use std::accumulate for these, because that doesn't work with
     * C++14/gcc7 when you are accumulating into a different type (e.g. from a
     * set of blocks into an int).
     */
    uint count = 0;
    std::for_each(cache_i_blocks.begin(), cache_i_blocks.end(), [&](const auto& b) {
      count += (b->is_out_of_sight() || b->dloc() == dcenter);
    });

    std::string accum;
    std::for_each(cache_i_blocks.begin(), cache_i_blocks.end(), [&](const auto& b) {
      accum += "b" + std::to_string(b->id()) + "->fb" +
               std::to_string(b->robot_id()) + ",";
    });
    ER_DEBUG("Block carry statuses: [%s]", accum.c_str());

    accum = "";
    std::for_each(cache_i_blocks.begin(), cache_i_blocks.end(), [&](const auto& b) {
      accum += "b" + std::to_string(b->id()) + "->" + b->dloc().to_str() + ",";
    });
    ER_DEBUG("Block locations: [%s]", accum.c_str());

    ER_ASSERT(cache_i_blocks.size() - count < repr::base_cache::kMinBlocks,
              "For new cache @%s/%s: %zu blocks SHOULD be "
              "available, but only %zu are (min=%zu)",
              loc.to_str().c_str(),
              dcenter.to_str().c_str(),
              cache_i_blocks.size() - count,
              cache_i_blocks.size(),
              repr::base_cache::kMinBlocks);
    return boost::optional<ds::block_vector>();
  }
  if (cache_i_blocks.size() < mc_cache_config.static_.size) {
    ER_WARN(
        "Not enough free blocks to meet min size for new cache@%s/%s (%zu < "
        "%u)",
        loc.to_str().c_str(),
        dcenter.to_str().c_str(),
        cache_i_blocks.size(),
        mc_cache_config.static_.size);
    return boost::optional<ds::block_vector>();
  }
  return boost::make_optional(cache_i_blocks);
} /* calc_blocks_for_cache_i() */

void static_cache_manager::post_creation_blocks_absorb(
    const ds::cache_vector& caches,
    const ds::block_vector& blocks) {
  for (auto& b : blocks) {
    for (auto& c : caches) {
      if (!c->contains_block(b) && c->xspan().overlaps_with(b->xspan()) &&
          c->yspan().overlaps_with(b->yspan())) {
        events::cell_empty_visitor empty(b->dloc());
        empty.visit(arena_grid()->access<arena_grid::kCell>(b->dloc()));
        events::free_block_drop_visitor op(
            b,
            rmath::dvec2uvec(c->rloc(), arena_grid()->resolution()),
            arena_grid()->resolution());
        op.visit(arena_grid()->access<arena_grid::kCell>(op.x(), op.y()));
        c->block_add(b);
        ER_INFO("Hidden block%d added to cache%d", b->id(), c->id());
      }
    } /* for(&c..) */
  }   /* for(&b..) */
} /* post_creation_blocks_absorb() */

NS_END(depth1, support, fordyca);
