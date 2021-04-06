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
#include "fordyca/support/d1/static_cache_manager.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/free_blocks_calculator.hpp"
#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/arena/operations/free_block_pickup.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/spatial/conflict_checker.hpp"
#include "cosm/spatial/dimension_checker.hpp"

#include "fordyca/math/cache_respawn_probability.hpp"
#include "fordyca/support/d1/static_cache_creator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, d1);
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
      ER_CLIENT_INIT("fordyca.support.d1.static_cache_manager"),
      mc_cache_config(*config),
      mc_cache_locs(cache_locs),
      m_rng(rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<cads::acache_vectoro>
static_cache_manager::create(const cache_create_ro_params& c_params,
                             const cds::block3D_vectorno& c_all_blocks,
                             bool initial) {
  ER_DEBUG("(Re)-Creating static cache(s)");
  ER_ASSERT(mc_cache_config.static_.size >= carepr::base_cache::kMinBlocks,
            "Static cache size %u < minimum %zu",
            mc_cache_config.static_.size,
            carepr::base_cache::kMinBlocks);

  auto to_use = blocks_alloc(c_params.current_caches, c_all_blocks);

  if (std::any_of(to_use.begin(), to_use.end(), [&](const auto& cache_i) {
        return cache_i.second.empty();
      })) {
    ER_WARN("Unable to create all static cache(s): Not enough free blocks "
            "(n_caches=%zu)",
            c_params.current_caches.size());
    return boost::optional<cads::acache_vectoro>();
  }

  /* (re)-create the caches */
  using checker = cspatial::dimension_checker;
  auto even_multiple = checker::even_multiple(arena_map()->grid_resolution(),
                                              mc_cache_config.dimension);
  auto odd_dsize =
      checker::odd_dsize(arena_map()->grid_resolution(), even_multiple);

  static_cache_creator creator(&arena_map()->decoratee(),
                               arena_map()->block_distributor(),
                               mc_cache_locs,
                               odd_dsize);

  static_cache_creator::creation_result res = creator.create_all(c_params,
                                                                 to_use,
                                                                 initial);

  /* Configure cache extents */
  creator.cache_extents_configure(res.created);

  /* update bloctree */
  bloctree_update(res.created);

  /* verify the created caches */
  cads::acache_vectorro sanity_caches;
  std::transform(res.created.begin(),
                 res.created.end(),
                 std::back_inserter(sanity_caches),
                 [&](const auto& c) { return c.get(); });
  auto free_blocks = carena::free_blocks_calculator(initial)(c_all_blocks,
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
    const cds::block3D_vectorno& c_all_blocks,
    size_t n_harvesters,
    size_t n_collectors) {
  math::cache_respawn_probability p(mc_cache_config.static_.respawn_scale_factor);

  if (p.calc(n_harvesters, n_collectors) >= m_rng->uniform(0.0, 1.0)) {
    return create(c_params, c_all_blocks, false);
  } else {
    return boost::optional<cads::acache_vectoro>();
  }
} /* create_conditional() */

ds::block_alloc_map static_cache_manager::blocks_alloc(
    const cads::acache_vectorno& c_existing_caches,
    const cds::block3D_vectorno& c_all_blocks) const {
  ds::block_alloc_map allocs;
  for (size_t i = 0; i < mc_cache_locs.size(); ++i) {
    if (auto cache_i = cache_i_blocks_alloc(c_existing_caches,
                                            allocs,
                                            c_all_blocks,
                                            mc_cache_locs[i],
                                            carepr::base_cache::kMinBlocks)) {
      allocs[i] = *cache_i;
    } else {
      allocs[i] = {};
    }
    ER_DEBUG("Alloc_blocks=[%s] for cache%zu@%s",
             rcppsw::to_string(allocs[i]).c_str(),
             i,
             mc_cache_locs[i].to_str().c_str());
  } /* for(i..) */
  return allocs;
} /* blocks_alloc() */

boost::optional<cds::block3D_vectorno> static_cache_manager::cache_i_blocks_alloc(
    const cads::acache_vectorno& c_existing_caches,
    const ds::block_alloc_map& c_alloc_map,
    const cds::block3D_vectorno& c_all_blocks,
    const rmath::vector2d& c_center,
    size_t n_blocks) const {
  cds::block3D_vectorno cache_i_blocks;

  /*
     * Initial allocation.
     *
     * Note that the calculations for membership are ordered from least to most
     * computationally expensive to compute, so don't reorder them willy-nilly.
    */
  std::copy_if(
      c_all_blocks.begin(),
      c_all_blocks.end(),
      std::back_inserter(cache_i_blocks),
      [&](const auto* b) {
        /* don't have enough blocks yet */
        return (cache_i_blocks.size() < n_blocks) &&
            /* not carried by robot */
            !b->is_carried_by_robot() &&
            /*
             * Not already on a cell where the cache will be re-created, or
             * on the cell where ANOTHER cache *might* be recreated.
             */
               std::all_of(mc_cache_locs.begin(),
                           mc_cache_locs.end(),
                           [&](const auto& loc) {
                             auto dloc = rmath::dvec2zvec(
                                 loc, arena_map()->grid_resolution().v());
                             return b->danchor2D() != dloc;
                           }) &&

               /* not already allocated for a different cache */
               !c_alloc_map.contains(b) &&

               std::all_of(c_existing_caches.begin(),
                           c_existing_caches.end(),
                           [&](const auto& c) { return !c->contains_block(b); });
      });

  ER_DEBUG("Cache initial allocation: %s (%zu)",
           rcppsw::to_string(cache_i_blocks).c_str(),
           cache_i_blocks.size());

  /* add blocks which will end up being under the created cache */
  rmath::vector2d dims(mc_cache_config.dimension.v(),
                       mc_cache_config.dimension.v());
  auto ranchor = c_center - dims / 2.0;
  std::copy_if(
      c_all_blocks.begin(),
      c_all_blocks.end(),
      std::back_inserter(cache_i_blocks),
      [&](const auto* b) {
        auto status = cspatial::conflict_checker::placement2D(ranchor, dims, b);

        /*
         * Only add blocks that:
         *
         * - Overlap with the extent of the to be created cache
         * - Are not already allocated to this cache
         * - Are not already allocated to another cache
         */
        return status.x && status.y &&
               cache_i_blocks.end() ==
                   std::find(cache_i_blocks.begin(), cache_i_blocks.end(), b) &&

               !c_alloc_map.contains(b);
      });

  ER_DEBUG("Cache allocation including hidden blocks: %s (%zu)",
           rcppsw::to_string(cache_i_blocks).c_str(),
           cache_i_blocks.size());
  if (!cache_i_blocks_alloc_check(cache_i_blocks, c_center)) {
    return boost::optional<cds::block3D_vectorno>();
  }
  return boost::make_optional(cache_i_blocks);
} /* cache_i_blocks_alloc() */

bool static_cache_manager::cache_i_blocks_alloc_check(
    const cds::block3D_vectorno& cache_i_blocks,
    const rmath::vector2d& c_center) const {
  if (cache_i_blocks.size() < carepr::base_cache::kMinBlocks) {
    auto dcenter = rmath::dvec2zvec(c_center, arena_map()->grid_resolution().v());

    /*
     * Cannot use std::accumulate for these, because that doesn't work with
     * C++14/gcc7 when you are accumulating into a different type (e.g. from a
     * set of blocks into an int).
     */
    size_t count = 0;
    std::for_each(
        cache_i_blocks.begin(), cache_i_blocks.end(), [&](const auto& b) {
          count += (b->is_out_of_sight() || b->danchor2D() == dcenter);
        });

    std::string accum;
    std::for_each(
        cache_i_blocks.begin(), cache_i_blocks.end(), [&](const auto& b) {
          accum += "b" + rcppsw::to_string(b->id()) + "->fb" +
                   rcppsw::to_string(b->md()->robot_id()) + ",";
        });
    ER_TRACE("Cache i alloc_blocks carry statuses: [%s]", accum.c_str());

    accum = "";
    std::for_each(
        cache_i_blocks.begin(), cache_i_blocks.end(), [&](const auto& b) {
          accum += "b" + rcppsw::to_string(b->id()) + "->" +
                   b->danchor2D().to_str() + ",";
        });
    ER_TRACE("Cache i alloc_blocks locs: [%s]", accum.c_str());

    ER_ASSERT(cache_i_blocks.size() - count < carepr::base_cache::kMinBlocks,
              "For new cache @%s: %zu blocks SHOULD be "
              "available, but only %zu are (min=%zu)",
              rcppsw::to_string(c_center).c_str(),
              cache_i_blocks.size() - count,
              cache_i_blocks.size(),
              carepr::base_cache::kMinBlocks);
    return false;
  }
  if (cache_i_blocks.size() < mc_cache_config.static_.size) {
    ER_WARN("Not enough free blocks to meet min size for new cache@/%s (%zu < "
            "%u)",
            rcppsw::to_string(c_center).c_str(),
            cache_i_blocks.size(),
            mc_cache_config.static_.size);
    return false;
  }
  return true;
} /* cache_i_blocks_alloc_check() */

NS_END(d1, support, fordyca);
