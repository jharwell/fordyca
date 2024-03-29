/**
 * \file dynamic_cache_creator.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/argos/support/d2/dynamic_cache_creator.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/operations/cache_extent_clear.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/repr/sim_block3D.hpp"
#include "cosm/spatial/conflict_checker.hpp"

#include "fordyca/argos/support/caches/creation_verifier.hpp"
#include "fordyca/argos/support/d2/cache_center_calculator.hpp"
#include "fordyca/events/cell2D_empty.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support, d2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dynamic_cache_creator::dynamic_cache_creator(const params* const p,
                                             rmath::rng* rng)
    : base_creator(p->map, p->cache_dim),
      ER_CLIENT_INIT("fordyca.argos.support.d2.dynamic_cache_creator"),
      mc_min_blocks(p->min_blocks),
      mc_min_dist(p->min_dist),
      mc_strict_constraints(p->strict_constraints),
      m_rng(rng),
      m_map(p->map) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
dynamic_cache_creator::creation_result
dynamic_cache_creator::create_all(const fascaches::create_ro_params& c_params,
                                  cds::block3D_vectorno&& usable_blocks,
                                  cds::block3D_htno&& absorbable_blocks) {
  creation_result res;

  ER_DEBUG("Creating caches: min_dist=%f,min_blocks=%u,usable_blocks=[%s] "
           "(%zu),absorbable_blocks=[%s] (%zu)",
           mc_min_dist.v(),
           mc_min_blocks,
           rcppsw::to_string(usable_blocks).c_str(),
           usable_blocks.size(),
           rcppsw::to_string(absorbable_blocks).c_str(),
           absorbable_blocks.size());

  cds::block3D_vectorno all_blocks;
  std::transform(absorbable_blocks.begin(),
                 absorbable_blocks.end(),
                 std::back_inserter(all_blocks),
                 [&](const auto& pair) { return pair.second; });

  for (auto anchor_it = usable_blocks.begin();
       anchor_it != usable_blocks.end();) {
    auto cache_i_initial = cache_i_blocks_alloc(usable_blocks, anchor_it);
    std::advance(anchor_it, cache_i_initial.size());
    ER_DEBUG("Removed %zu allocated blocks from usable vector",
             cache_i_initial.size());
    if (cache_i_initial.size() < mc_min_blocks) {
      continue;
    }

    auto cache_i = cache_i_create(
        c_params, cache_i_initial, absorbable_blocks, &res.created);

    if (cache_i.status) {
      cads::acache_vectorro sanity_caches;
      std::transform(res.created.begin(),
                     res.created.end(),
                     std::back_inserter(sanity_caches),
                     [&](const auto& c) { return c.get(); });
      sanity_caches.push_back(cache_i.cache.get());

      auto verifier = fascaches::creation_verifier(m_map,
                                                   cache_dim(),
                                                   mc_strict_constraints);
      if (!verifier.verify_single(cache_i.cache.get(),
                                  sanity_caches,
                                  all_blocks,
                                  c_params.clusters)) {
        cache_delete(cache_i);
      } else {
        res.created.push_back(cache_i.cache);
        /*
         * Remove all the blocks successfully used during creation from the set
         * of blocks which can be absorbed as part of subsequent successful
         * cache creations.
         */
        for (auto* block : cache_i.used) {
          absorbable_blocks.erase(block->id());
        } /* for(*block..) */
      }
    } else {
      ++res.n_discarded;
    }
  } /* for(anchor_it..) */
  return res;
} /* create_all() */

dynamic_cache_creator::cache_i_result dynamic_cache_creator::cache_i_create(
    const fascaches::create_ro_params& c_params,
    const cds::block3D_vectorno& c_alloc_blocks,
    const cds::block3D_htno& c_absorbable_blocks,
    cads::acache_vectoro* created) {
  cads::acache_vectorno c_avoid =
      avoidance_caches_calc(c_params.current_caches, *created);

  /* First, try find a conflict free cache center (host cell) */
  auto calculator = cache_center_calculator(
      &map()->decoratee(), cache_dim(), m_map->nests(), c_params.clusters);
  auto center = calculator(c_alloc_blocks, c_avoid, m_rng);
  if (!center) {
    return {};
  }

  /*
   * If we found a conflict free cache center, we need to find all the free
   * blocks within the extent of the cache-to-be, and add them into the block
   * list for the new cache (absorption).
   */
  auto absorb_blocks = cache_i_alloc_from_absorbable(
      c_absorbable_blocks, c_alloc_blocks, *center, cache_dim());
  ER_DEBUG("Absorb blocks=[%s]", rcppsw::to_string(absorb_blocks).c_str());

  /* blocks for cache i = allocated blocks + absorb blocks */
  cds::block3D_vectorno cache_i_blocks(c_alloc_blocks.begin(),
                                       c_alloc_blocks.end());
  std::transform(absorb_blocks.begin(),
                 absorb_blocks.end(),
                 std::back_inserter(cache_i_blocks),
                 [&](const auto& pair) { return pair.second; });

  ER_DEBUG("Cache blocks=[%s]", rcppsw::to_string(cache_i_blocks).c_str());

  auto cache =
      create_single_cache(*center, std::move(cache_i_blocks), c_params.t, false);
  return { true, std::move(cache), cache_i_blocks };
} /* cache_i_create() */

cds::block3D_vectorno dynamic_cache_creator::cache_i_blocks_alloc(
    const cds::block3D_vectorno& c_usable_blocks,
    cds::block3D_vectorno::iterator anchor_it) const {
  cds::block3D_vectorno cache_i_blocks;

  /*
   * No usable blocks left, or our anchor block has already been used.
   */
  if (c_usable_blocks.empty() || c_usable_blocks.end() == anchor_it) {
    return {};
  }

  /*
   * Add our anchor/target block to the list of blocks for the new cache. This
   * is OK to do even if there are no other blocks close enough to create a new
   * cache, because we have a minimum # blocks threshold that has to be met
   * anyway.
   */
  ER_TRACE("Add anchor block%d@%s/%s to src list",
           (*anchor_it)->id().v(),
           rcppsw::to_string((*anchor_it)->ranchor2D()).c_str(),
           rcppsw::to_string((*anchor_it)->danchor2D()).c_str());
  cache_i_blocks.push_back(*anchor_it);

  ++anchor_it;
  while (anchor_it != c_usable_blocks.end()) {
    auto* candidate = *anchor_it;
    ++anchor_it;
    /*
     * If we find a block that is close enough to our anchor/target block, then
     * add to the src list.
     */
    rtypes::spatial_dist to_block(
        (candidate->rcenter2D() - candidate->rcenter2D()).length());
    if (to_block <= mc_min_dist) {
      ER_ASSERT(std::find(cache_i_blocks.begin(),
                          cache_i_blocks.end(),
                          candidate) == cache_i_blocks.end(),
                "Block%d already on src list",
                candidate->id().v());
      ER_TRACE("Add block%d@%s/%s to src list",
               candidate->id().v(),
               rcppsw::to_string(candidate->ranchor2D()).c_str(),
               rcppsw::to_string(candidate->danchor2D()).c_str());
      cache_i_blocks.push_back(candidate);
    }
  } /* while() */
  return cache_i_blocks;
} /* cache_i_blocks_alloc() */

cds::block3D_htno dynamic_cache_creator::cache_i_alloc_from_absorbable(
    const cds::block3D_htno& c_absorbable_blocks,
    const cds::block3D_vectorno& c_cache_i_blocks,
    const rmath::vector2d& c_center,
    const rtypes::spatial_dist& c_cache_dim) const {
  cds::block3D_htno absorb_blocks;
  rmath::vector2d cache_dim(c_cache_dim.v(), c_cache_dim.v());
  using checker = cspatial::conflict_checker;

  std::copy_if(c_absorbable_blocks.begin(),
               c_absorbable_blocks.end(),
               std::inserter(absorb_blocks, absorb_blocks.begin()),
               [&](const auto& pair) RCPPSW_PURE {
                 auto* block = pair.second;
                 auto status = checker::placement2D(
                     c_center - cache_dim / 2.0, cache_dim, block);

                 return
                     /* block overlaps cache i */
                     status.x && status.y &&
                     /* block is not already allocated to cache i */
                     c_cache_i_blocks.end() == std::find(c_cache_i_blocks.begin(),
                                                         c_cache_i_blocks.end(),
                                                         block);
               });
  return absorb_blocks;
} /* cache_i_alloc_from_absorbable() */

cads::acache_vectorno dynamic_cache_creator::avoidance_caches_calc(
    const cads::acache_vectorno& c_previous_caches,
    const cads::acache_vectoro& c_created_caches) const {
  cads::acache_vectorno avoid = c_previous_caches;
  for (const auto& c : c_created_caches) {
    avoid.push_back(c.get());
  } /* for(&c..) */
  return avoid;
} /* avoidance_caches_calc() */

void dynamic_cache_creator::cache_delete(const cache_i_result& cache_i) {
  ER_INFO("Delete (badly) created cache%d", cache_i.cache->id().v());

  /*
   * Clear out cache host cell. Note that we STILL need to clear the
   * CACHE_EXTENT, even though that is only set upon successful creation. If
   * this function is called as a result of a cache being created with a bad
   * center, then just clearing the "center" cell is not guaranteed to reset the
   * arena to a good state--we need to clear all cache extent cells.
   */
  events::cell2D_empty_visitor hc(cache_i.cache->dcenter2D());
  caops::cache_extent_clear_visitor ec(cache_i.cache.get());

  hc.visit(m_map->decoratee());
  ec.visit(m_map->decoratee());

  /* redistribute blocks */
  for (auto* b : cache_i.used) {
    /*
     * We are actually holding zero locks, but we don't need to take any
     * since dynamic cache creation always happens in a non-concurrent
     * context.
     */
    m_map->distribute_single_block(b, carena::locking::ekALL_HELD);
  } /* for(*b..) */
} /* cache_delete() */

NS_END(d2, support, argos, fordyca);
