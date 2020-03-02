/**
 * \file static_cache_manager.cpp
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
#include "fordyca/support/depth1/static_cache_manager.hpp"

#include "cosm/ds/arena_grid.hpp"
#include "cosm/events/cell2D_empty.hpp"
#include "cosm/foraging/repr/arena_cache.hpp"
#include "cosm/repr/base_block2D.hpp"
#include "cosm/foraging/events/arena_free_block_drop.hpp"

#include "fordyca/math/cache_respawn_probability.hpp"
#include "fordyca/support/depth1/static_cache_creator.hpp"
#include "fordyca/support/utils/loop_utils.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);
using cds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
static_cache_manager::static_cache_manager(
    const config::caches::caches_config* config,
    cds::arena_grid* const arena_grid,
    const std::vector<rmath::vector2d>& cache_locs,
    rmath::rng* rng)
    : base_cache_manager(arena_grid),
      ER_CLIENT_INIT("fordyca.support.depth1.static_cache_manager"),
      mc_cache_config(*config),
      mc_cache_locs(cache_locs),
      m_rng(rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<cfds::cache_vector> static_cache_manager::create(
    const cache_create_ro_params& c_params,
    const cfds::block_vector& c_alloc_blocks) {
  ER_DEBUG("(Re)-Creating static cache(s)");
  ER_ASSERT(mc_cache_config.static_.size >= cfrepr::base_cache::kMinBlocks,
            "Static cache size %u < minimum %zu",
            mc_cache_config.static_.size,
            cfrepr::base_cache::kMinBlocks);

  auto to_use = blocks_alloc(c_params.current_caches, c_alloc_blocks);
  if (!to_use) {
    ER_WARN(
        "Unable to create static cache(s): Not enough free blocks "
        "(n_caches=%zu,n_alloc_blocks=%zu)",
        c_params.current_caches.size(),
        c_alloc_blocks.size());
    return boost::optional<cfds::cache_vector>();
  }
  static_cache_creator creator(arena_grid(),
                               mc_cache_locs,
                               mc_cache_config.dimension);

  auto created = creator.create_all(c_params, *to_use);

  /*
   * Fix hidden blocks and update host cells. Host cell updating must be second,
   * otherwise the cache host cell will have a block as its entity!
   */
  post_creation_blocks_absorb(created, c_alloc_blocks);
  creator.update_host_cells(created);

  auto free_blocks = utils::free_blocks_calc(created, c_alloc_blocks);
  ER_ASSERT(
      creator.creation_sanity_checks(created, free_blocks, c_params.clusters),
      "One or more bad caches on creation");

  caches_created(created.size());
  return boost::make_optional(created);
} /* create() */

boost::optional<cfds::cache_vector> static_cache_manager::create_conditional(
    const cache_create_ro_params& c_params,
    const cfds::block_vector& c_alloc_blocks,
    uint n_harvesters,
    uint n_collectors) {
  math::cache_respawn_probability p(
      mc_cache_config.static_.respawn_scale_factor);

  if (p.calc(n_harvesters, n_collectors) >= m_rng->uniform(0.0, 1.0)) {
    return create(c_params, c_alloc_blocks);
  } else {
    return boost::optional<cfds::cache_vector>();
  }
} /* create_conditional() */

boost::optional<cfds::block_vector> static_cache_manager::blocks_alloc(
    const cfds::cache_vector& existing_caches,
    const cfds::block_vector& blocks) const {
  cfds::block_vector alloc_i;
  for (auto& loc : mc_cache_locs) {
    if (auto cache_i = cache_i_blocks_alloc(existing_caches,
                                            alloc_i,
                                            blocks,
                                            loc,
                                            cfrepr::base_cache::kMinBlocks)) {
      ER_DEBUG("Alloc_blocks=[%s] for cache@%s",
               rcppsw::to_string(*cache_i).c_str(),
               loc.to_str().c_str());
      alloc_i.insert(alloc_i.end(), cache_i->begin(), cache_i->end());
    }
  } /* for(&loc..) */

  if (alloc_i.empty()) {
    return boost::optional<cfds::block_vector>();
  } else {
    return boost::make_optional(alloc_i);
  }
} /* blocks_alloc() */

boost::optional<cfds::block_vector> static_cache_manager::cache_i_blocks_alloc(
    const cfds::cache_vector& existing_caches,
    const cfds::block_vector& allocated_blocks,
    const cfds::block_vector& all_blocks,
    const rmath::vector2d& loc,
    size_t n_blocks) const {
  cfds::block_vector cache_i_blocks;
  rmath::vector2u dcenter =
      rmath::dvec2uvec(loc, arena_grid()->resolution().v());
  std::copy_if(
      all_blocks.begin(),
      all_blocks.end(),
      std::back_inserter(cache_i_blocks),
      [&](const auto& b) {
        /* don't have enough blocks yet */
        return (cache_i_blocks.size() < n_blocks) &&
               /* not carried by robot */
               rtypes::constants::kNoUUID == b->robot_id() &&
               /* not already allocated for a different cache */
               allocated_blocks.end() == std::find(allocated_blocks.begin(),
                                                   allocated_blocks.end(),
                                                   b) &&
               /* not already in an existing cache */
               std::all_of(existing_caches.begin(),
                           existing_caches.end(),
                           [&](const auto& c) {
                             return !c->contains_block(b);
                           }) &&
               /* not already on a cell where cache will be re-created, or
                * on the cell where ANOTHER cache *might* be recreated */
               std::all_of(mc_cache_locs.begin(),
                           mc_cache_locs.end(),
                           [&](const auto& l) {
                             return b->dloc() !=
                                    rmath::dvec2uvec(
                                        l, arena_grid()->resolution().v());
                           });
      });

  if (cache_i_blocks.size() < cfrepr::base_cache::kMinBlocks) {
    /*
     * Cannot use std::accumulate for these, because that doesn't work with
     * C++14/gcc7 when you are accumulating into a different type (e.g. from a
     * set of blocks into an int).
     */
    uint count = 0;
    std::for_each(cache_i_blocks.begin(),
                  cache_i_blocks.end(),
                  [&](const auto& b) {
                    count += (b->is_out_of_sight() || b->dloc() == dcenter);
                  });

    std::string accum;
    std::for_each(cache_i_blocks.begin(),
                  cache_i_blocks.end(),
                  [&](const auto& b) {
                    accum += "b" + rcppsw::to_string(b->id()) + "->fb" +
                             rcppsw::to_string(b->robot_id()) + ",";
                  });
    ER_TRACE("Cache i alloc_blocks carry statuses: [%s]", accum.c_str());

    accum = "";
    std::for_each(cache_i_blocks.begin(),
                  cache_i_blocks.end(),
                  [&](const auto& b) {
                    accum += "b" + rcppsw::to_string(b->id()) + "->" +
                             b->dloc().to_str() + ",";
                  });
    ER_TRACE("Cache i alloc_blocks locs: [%s]", accum.c_str());

    ER_ASSERT(cache_i_blocks.size() - count < cfrepr::base_cache::kMinBlocks,
              "For new cache @%s/%s: %zu blocks SHOULD be "
              "available, but only %zu are (min=%zu)",
              loc.to_str().c_str(),
              dcenter.to_str().c_str(),
              cache_i_blocks.size() - count,
              cache_i_blocks.size(),
              cfrepr::base_cache::kMinBlocks);
    return boost::optional<cfds::block_vector>();
  }
  if (cache_i_blocks.size() < mc_cache_config.static_.size) {
    ER_WARN(
        "Not enough free blocks to meet min size for new cache@%s/%s (%zu < "
        "%u)",
        loc.to_str().c_str(),
        dcenter.to_str().c_str(),
        cache_i_blocks.size(),
        mc_cache_config.static_.size);
    return boost::optional<cfds::block_vector>();
  }
  return boost::make_optional(cache_i_blocks);
} /* cache_i_blocks_alloc() */

void static_cache_manager::post_creation_blocks_absorb(
    const cfds::cache_vector& caches,
    const cfds::block_vector& blocks) {
  for (auto& b : blocks) {
    for (auto& c : caches) {
      if (!c->contains_block(b) && c->xspan().overlaps_with(b->xspan()) &&
          c->yspan().overlaps_with(b->yspan())) {
        cevents::cell2D_empty_visitor empty(b->dloc());
        empty.visit(arena_grid()->access<arena_grid::kCell>(b->dloc()));
        /*
         * We are not REALLY holding all the arena map locks, but since cache
         * creation always happens AFTER all robot control steps have been run,
         * no locking is needed.
         */
        cfevents::arena_free_block_drop_visitor op(
            b,
            rmath::dvec2uvec(c->rloc(), arena_grid()->resolution().v()),
            arena_grid()->resolution(),
            cfds::arena_map_locking::ekALL_HELD);
        op.visit(arena_grid()->access<arena_grid::kCell>(op.x(), op.y()));
        c->block_add(b);
        ER_INFO("Hidden block%d added to cache%d", b->id().v(), c->id().v());
      }
    } /* for(&c..) */
  }   /* for(&b..) */
} /* post_creation_blocks_absorb() */

NS_END(depth1, support, fordyca);
