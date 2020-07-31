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

#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/arena/free_blocks_calculator.hpp"
#include "cosm/arena/operations/free_block_pickup.hpp"

#include "fordyca/math/cache_respawn_probability.hpp"
#include "fordyca/support/depth1/static_cache_creator.hpp"

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
    carena::caching_arena_map* const map,
    const std::vector<rmath::vector2d>& cache_locs,
    rmath::rng* rng)
    : base_cache_manager(map),
      ER_CLIENT_INIT("fordyca.support.depth1.static_cache_manager"),
      mc_cache_config(*config),
      mc_cache_locs(cache_locs),
      m_rng(rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<cads::acache_vectoro> static_cache_manager::create(
    const cache_create_ro_params& c_params,
    const cds::block3D_vectorno& c_alloc_blocks,
    bool initial) {
  ER_DEBUG("(Re)-Creating static cache(s)");
  ER_ASSERT(mc_cache_config.static_.size >= carepr::base_cache::kMinBlocks,
            "Static cache size %u < minimum %zu",
            mc_cache_config.static_.size,
            carepr::base_cache::kMinBlocks);

  auto to_use = blocks_alloc(c_params.current_caches, c_alloc_blocks);
  if (!to_use) {
    ER_WARN(
        "Unable to create static cache(s): Not enough free blocks "
        "(n_caches=%zu,n_alloc_blocks=%zu)",
        c_params.current_caches.size(),
        c_alloc_blocks.size());
    return boost::optional<cads::acache_vectoro>();
  }

  /* (re)-create the caches */
  auto dimension = dimension_check(mc_cache_config.dimension);

  static_cache_creator creator(&arena_map()->decoratee(),
                               mc_cache_locs,
                               dimension);

  static_cache_creator::creation_result res = creator.create_all(c_params,
                                                                 *to_use,
                                                                 initial);


  /* Fix hidden blocks and configure cache extents */
  post_creation_blocks_absorb(res.created, c_alloc_blocks);
  creator.configure_cache_extents(res.created);

  /* verify the created caches */
  cads::acache_vectorro sanity_caches;
  std::transform(res.created.begin(),
                 res.created.end(),
                 std::back_inserter(sanity_caches),
                 [&](const auto& c) {
                   return c.get();
                 });
  auto free_blocks = carena::free_blocks_calculator()(c_alloc_blocks,
                                                      sanity_caches);
  ER_ASSERT(creator.creation_sanity_checks(sanity_caches,
                                           free_blocks,
                                           c_params.clusters,
                                           arena_map()->nests()),
            "One or more bad caches on creation");

  caches_created(res.created.size());
  caches_discarded(res.n_discarded);

  return boost::make_optional(res.created);
} /* create() */

boost::optional<cads::acache_vectoro> static_cache_manager::create_conditional(
    const cache_create_ro_params& c_params,
    const cds::block3D_vectorno& c_alloc_blocks,
    size_t n_harvesters,
    size_t n_collectors) {
  math::cache_respawn_probability p(
      mc_cache_config.static_.respawn_scale_factor);

  if (p.calc(n_harvesters, n_collectors) >= m_rng->uniform(0.0, 1.0)) {
    return create(c_params, c_alloc_blocks, false);
  } else {
    return boost::optional<cads::acache_vectoro>();
  }
} /* create_conditional() */

boost::optional<cds::block3D_vectorno> static_cache_manager::blocks_alloc(
    const cads::acache_vectorno& existing_caches,
    const cds::block3D_vectorno& all_blocks) const {
  cds::block3D_vectorno alloc_i;
  for (auto& loc : mc_cache_locs) {
    auto dloc = rmath::dvec2zvec(loc, arena_map()->grid_resolution().v());
    if (auto cache_i = cache_i_blocks_alloc(existing_caches,
                                            alloc_i,
                                            all_blocks,
                                            dloc,
                                            carepr::base_cache::kMinBlocks)) {
      ER_DEBUG("Alloc_blocks=[%s] for cache@%s",
               rcppsw::to_string(*cache_i).c_str(),
               loc.to_str().c_str());
      alloc_i.insert(alloc_i.end(), cache_i->begin(), cache_i->end());
    }
  } /* for(&loc..) */

  if (alloc_i.empty()) {
    return boost::optional<cds::block3D_vectorno>();
  } else {
    return boost::make_optional(alloc_i);
  }
} /* blocks_alloc() */

boost::optional<cds::block3D_vectorno> static_cache_manager::cache_i_blocks_alloc(
    const cads::acache_vectorno& existing_caches,
    const cds::block3D_vectorno& allocated_blocks,
    const cds::block3D_vectorno& all_blocks,
    const rmath::vector2z& center,
    size_t n_blocks) const {
  cds::block3D_vectorno cache_i_blocks;
  std::copy_if(
      all_blocks.begin(),
      all_blocks.end(),
      std::back_inserter(cache_i_blocks),
      [&](const auto* b) {
        /* don't have enough blocks yet */
        return (cache_i_blocks.size() < n_blocks) &&
               /* not carried by robot */
               rtypes::constants::kNoUUID == b->md()->robot_id() &&
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
               /*
                * Not already on a cell where the cache will be re-created, or
                * on the cell where ANOTHER cache *might* be recreated.
                */
               std::all_of(mc_cache_locs.begin(),
                           mc_cache_locs.end(),
                           [&](const auto& loc) {
                             auto dloc = rmath::dvec2zvec(loc,
                                                          arena_map()->grid_resolution().v());
                             return b->danchor2D() != dloc;
                           });
      });

  if (cache_i_blocks.size() < carepr::base_cache::kMinBlocks) {
    /*
     * Cannot use std::accumulate for these, because that doesn't work with
     * C++14/gcc7 when you are accumulating into a different type (e.g. from a
     * set of blocks into an int).
     */
    size_t count = 0;
    std::for_each(cache_i_blocks.begin(),
                  cache_i_blocks.end(),
                  [&](const auto& b) {
                    count += (b->is_out_of_sight() || b->danchor2D() == center);
                  });

    std::string accum;
    std::for_each(cache_i_blocks.begin(),
                  cache_i_blocks.end(),
                  [&](const auto& b) {
                    accum += "b" + rcppsw::to_string(b->id()) + "->fb" +
                             rcppsw::to_string(b->md()->robot_id()) + ",";
                  });
    ER_TRACE("Cache i alloc_blocks carry statuses: [%s]", accum.c_str());

    accum = "";
    std::for_each(cache_i_blocks.begin(),
                  cache_i_blocks.end(),
                  [&](const auto& b) {
                    accum += "b" + rcppsw::to_string(b->id()) + "->" +
                             b->danchor2D().to_str() + ",";
                  });
    ER_TRACE("Cache i alloc_blocks locs: [%s]", accum.c_str());

    ER_ASSERT(cache_i_blocks.size() - count < carepr::base_cache::kMinBlocks,
              "For new cache @%s: %zu blocks SHOULD be "
              "available, but only %zu are (min=%zu)",
              rcppsw::to_string(center).c_str(),
              cache_i_blocks.size() - count,
              cache_i_blocks.size(),
              carepr::base_cache::kMinBlocks);
    return boost::optional<cds::block3D_vectorno>();
  }
  if (cache_i_blocks.size() < mc_cache_config.static_.size) {
    ER_WARN(
        "Not enough free blocks to meet min size for new cache@/%s (%zu < "
        "%u)",
        rcppsw::to_string(center).c_str(),
        cache_i_blocks.size(),
        mc_cache_config.static_.size);
    return boost::optional<cds::block3D_vectorno>();
  }
  return boost::make_optional(cache_i_blocks);
} /* cache_i_blocks_alloc() */

void static_cache_manager::post_creation_blocks_absorb(
    const cads::acache_vectoro& caches,
    const cds::block3D_vectorno& blocks) {
  for (auto& b : blocks) {
    for (auto& c : caches) {
      if (!c->contains_block(b) && c->xrspan().overlaps_with(b->xrspan()) &&
          c->yrspan().overlaps_with(b->yrspan())) {
        auto pickup = caops::free_block_pickup_visitor::by_arena(b);

        /* pickup the block: mark host cell as empty and clear extent */
        pickup.visit(*arena_map());

        /*
         * We are not REALLY holding all the arena map locks, but since cache
         * creation always happens AFTER all robot control steps have been run,
         * no locking is needed.
         */
        caops::free_block_drop_visitor op(
            b,
            c->dcenter2D(),
            arena_map()->grid_resolution(),
            carena::arena_map_locking::ekALL_HELD);
        op.visit(arena_map()->access<arena_grid::kCell>(op.coord()));
        op.visit(*b);
        c->block_add(b);
        ER_INFO("Hidden block%d added to cache%d", b->id().v(), c->id().v());
      }
    } /* for(&c..) */
  }   /* for(&b..) */
} /* post_creation_blocks_absorb() */

NS_END(depth1, support, fordyca);
