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
#include "fordyca/support/cache_creation_verifier.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, d1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
static_cache_manager::static_cache_manager(
    const config::caches::caches_config* config,
    carena::caching_arena_map* const map,
    const std::vector<rmath::vector2d>& cache_locs,
    rmath::rng* rng)
    : base_cache_manager(config, map),
      ER_CLIENT_INIT("fordyca.support.d1.static_cache_manager"),
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
  ER_ASSERT(config()->static_.size >= carepr::base_cache::kMinBlocks,
            "Static cache size %u < minimum %zu",
            config()->static_.size,
            carepr::base_cache::kMinBlocks);

  auto usable_cb = std::bind(&static_cache_manager::block_alloc_usable_filter,
                             this,
                             std::placeholders::_1,
                             std::placeholders::_2,
                             std::placeholders::_3);
  auto absorbable_cb = std::bind(&static_cache_manager::block_alloc_absorbable_filter,
                                 this,
                                 std::placeholders::_1,
                                 std::placeholders::_2,
                                 std::placeholders::_3);
  if (auto for_creation = creation_blocks_alloc(c_all_blocks,
                                                c_params.current_caches,
                                                c_params.clusters,
                                                usable_cb,
                                                absorbable_cb)) {
    auto allocated = blocks_alloc(for_creation->usable,
                                  for_creation->absorbable);

    /* (re)-create the caches */
    using checker = cspatial::dimension_checker;
    auto even_multiple = checker::even_multiple(arena_map()->grid_resolution(),
                                                config()->dimension);

    auto odd_dsize = checker::odd_dsize(arena_map()->grid_resolution(),
                                        even_multiple);

    static_cache_creator creator(arena_map(), mc_cache_locs, odd_dsize);

    auto res = creator.create_all(c_params,
                                  std::move(allocated),
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
    auto free_blocks =
        carena::free_blocks_calculator(initial)(c_all_blocks, sanity_caches);
    auto verifier = cache_creation_verifier(arena_map(),
                                            odd_dsize,
                                            config()->strict_constraints);
    ER_ASSERT(verifier.sanity_checks(sanity_caches,
                                     free_blocks,
                                     c_params.clusters,
                                     arena_map()->nests()),
              "One or more caches failed verification");

    caches_created(res.created.size());
    caches_discarded(res.n_discarded);

    return boost::make_optional(res.created);
  } else {
    ER_WARN("Could not create static caches: bad block allocation");
    return boost::none;
  }
} /* create() */

boost::optional<cads::acache_vectoro> static_cache_manager::create_conditional(
    const cache_create_ro_params& c_params,
    const cds::block3D_vectorno& c_all_blocks,
    size_t n_harvesters,
    size_t n_collectors) {
  math::cache_respawn_probability p(config()->static_.respawn_scale_factor);

  if (p.calc(n_harvesters, n_collectors) >= m_rng->uniform(0.0, 1.0)) {
    return create(std::move(c_params), c_all_blocks, false);
  } else {
    return boost::optional<cads::acache_vectoro>();
  }
} /* create_conditional() */

ds::block_alloc_map static_cache_manager::blocks_alloc(
    const cds::block3D_vectorno& c_usable_blocks,
    const cds::block3D_htno& c_absorbable_blocks) const {
  ds::block_alloc_map alloc_map;
  for (size_t i = 0; i < mc_cache_locs.size(); ++i) {
    if (auto cache_i = cache_i_blocks_alloc(c_usable_blocks,
                                            c_absorbable_blocks,
                                            alloc_map,
                                            mc_cache_locs[i],
                                            i,
                                            carepr::base_cache::kMinBlocks)) {
      alloc_map[i] = *cache_i;
    } else {
      alloc_map[i] = {};
    }
    ER_DEBUG("Alloc_blocks=[%s] for cache%zu@%s",
             rcppsw::to_string(alloc_map[i]).c_str(),
             i,
             mc_cache_locs[i].to_str().c_str());
  } /* for(i..) */
  return alloc_map;
} /* blocks_alloc() */

boost::optional<cds::block3D_vectorno> static_cache_manager::cache_i_blocks_alloc(
    const cds::block3D_vectorno& c_usable_blocks,
    const cds::block3D_htno& c_absorbable_blocks,
    const ds::block_alloc_map& c_alloc_map,
    const rmath::vector2d& c_center,
    RCPPSW_UNUSED size_t cache_index,
    size_t required_blocks) const {
  /* initial allocation */
  auto alloc_blocks = cache_i_alloc_from_usable(c_usable_blocks,
                                                 c_alloc_map,
                                                 required_blocks);
  ER_DEBUG("Cache%zu initial allocation: %s (%zu)",
           cache_index,
           rcppsw::to_string(alloc_blocks).c_str(),
           alloc_blocks.size());

  /*
   * Find all the free blocks within the extent of the cache-to-be, and add them
   * into the block list for the new cache (absorption).
   */
  auto absorb_blocks = cache_i_alloc_from_absorbable(c_absorbable_blocks,
                                                     alloc_blocks,
                                                     c_alloc_map,
                                                     c_center);

  /* blocks for cache i = initially allocated blocks + absorbed blocks */
  cds::block3D_vectorno cache_i_blocks(alloc_blocks.begin(),
                                       alloc_blocks.end());
  std::transform(absorb_blocks.begin(),
                 absorb_blocks.end(),
                 std::back_inserter(cache_i_blocks),
                 [&](const auto& pair) { return pair.second; });

  ER_DEBUG("Cache%zu allocation after absorbtion: %s (%zu)",
           cache_index,
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
  if (cache_i_blocks.size() < config()->static_.size) {
    ER_WARN("Not enough free blocks to meet min size for new cache@/%s (%zu < "
            "%u)",
            rcppsw::to_string(c_center).c_str(),
            cache_i_blocks.size(),
            config()->static_.size);
    return false;
  }
  return true;
} /* cache_i_blocks_alloc_check() */

cds::block3D_vectorno static_cache_manager::cache_i_alloc_from_usable(
    const cds::block3D_vectorno& c_usable_blocks,
    const ds::block_alloc_map& c_alloc_map,
    size_t required_blocks) const {
  cds::block3D_vectorno cache_i_blocks;
  rmath::vector2d cache_dim(config()->dimension.v(), config()->dimension.v());
  using checker = cspatial::conflict_checker;
  /*
   * Note that the calculations for membership are ordered from least to most
   * computationally expensive to compute, so don't reorder them willy-nilly.
   */
  std::copy_if(
      c_usable_blocks.begin(),
      c_usable_blocks.end(),
      std::back_inserter(cache_i_blocks),
      [&](const auto& b) {
        return
            /* don't have enough blocks yet */
            (cache_i_blocks.size() < required_blocks) &&

            /*
             * Not already within the extent of ANY cache (including the one we
             * are allocating blocks for). Blocks can be dropped within cache
             * extents in (for example) RN scenarios during block distribution
             * or due to task abort when in between when a given cache is
             * depleted and when it is re-created.
             *
             * Adding blocks within the extent of the current cache we are
             * calculating the allocation for is done during the absorbtion
             * phase later.
             */
            std::all_of(mc_cache_locs.begin(),
                        mc_cache_locs.end(),
                        [&](const auto& center) {
                          auto status = checker::placement2D(
                              center - cache_dim / 2.0, cache_dim, b);

                          return !(status.x && status.y);
                        }) &&

            /* not already allocated for a different cache */
            !c_alloc_map.contains(b);
      });
  return cache_i_blocks;
} /* cache_i_alloc_from_usable() */

bool static_cache_manager::block_alloc_usable_filter(
    const crepr::base_block3D* block,
    const cads::acache_vectorno& existing_caches,
    const cfds::block3D_cluster_vectorro&) const {
  cds::block3D_vectorno cache_i_blocks;

  /*
   * Note that the calculations for membership are ordered from least to most
   * computationally expensive to compute, so don't reorder them willy-nilly.
   */
  return
      /* blocks cannot be carried by a robot */
      !block->is_carried_by_robot() &&

      /* blocks cannot be in existing caches */
      std::all_of(existing_caches.begin(),
                  existing_caches.end(),
                  [&](const auto& c) { return !c->contains_block(block); });
} /* block_alloc_usable_filter() */


bool static_cache_manager::block_alloc_absorbable_filter(
    const crepr::base_block3D* block,
    const cads::acache_vectorno& existing_caches,
    const cfds::block3D_cluster_vectorro&) {
  /* blocks cannot be carried by a robot */
  return !block->is_carried_by_robot() &&
      /* Blocks cannot be in existing caches */
      std::all_of(existing_caches.begin(),
                  existing_caches.end(),
                  [&](const auto& c) { return !c->contains_block(block); });
}/* block_alloc_absorbable_filter() */

cds::block3D_htno static_cache_manager::cache_i_alloc_from_absorbable(
    const cds::block3D_htno& c_absorbable_blocks,
    const cds::block3D_vectorno& c_cache_i_blocks,
    const ds::block_alloc_map& c_alloc_map,
    const rmath::vector2d& c_center) const {
  cds::block3D_htno absorb_blocks;
  rmath::vector2d cache_dim(config()->dimension.v(), config()->dimension.v());
  using checker = cspatial::conflict_checker;

  /*
   * Note that the calculations for membership are ordered from least to most
   * computationally expensive to compute, so don't reorder them willy-nilly.
v   */
  std::copy_if(
      c_absorbable_blocks.begin(),
      c_absorbable_blocks.end(),
      std::inserter(absorb_blocks, absorb_blocks.begin()),
      [&](const auto& pair) {
        auto* block = pair.second;
        auto status = checker::placement2D(
            c_center - cache_dim / 2.0, cache_dim, block);

        return
            /* block overlaps with extent of cache-to-be */
            status.x && status.y &&
            /* block is not already allocated to cache-to-be */
            c_cache_i_blocks.end() == std::find(c_cache_i_blocks.begin(),
                                                c_cache_i_blocks.end(),
                                                block) &&

            /* block is not already allocated to another cache-to-be */
            !c_alloc_map.contains(block);
      });
  return absorb_blocks;
}/* cache_i_alloc_from_absorbable() */

NS_END(d1, support, fordyca);
