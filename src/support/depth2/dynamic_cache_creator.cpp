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
#include "fordyca/support/depth2/dynamic_cache_creator.hpp"

#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/foraging/utils/utils.hpp"

#include "fordyca/events/cell2D_empty.hpp"
#include "fordyca/support/depth2/cache_center_calculator.hpp"
#include "fordyca/support/utils/loop_utils.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dynamic_cache_creator::dynamic_cache_creator(const params* const p,
                                             rmath::rng* rng)
    : base_cache_creator(&p->map->decoratee(), p->cache_dim),
      ER_CLIENT_INIT("fordyca.support.depth2.dynamic_cache_creator"),
      mc_strict_constraints(p->strict_constraints),
      mc_min_blocks(p->min_blocks),
      mc_min_dist(p->min_dist),
      m_rng(rng),
      m_map(p->map) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
dynamic_cache_creator::creation_result dynamic_cache_creator::create_all(
    const cache_create_ro_params& c_params,
    const cds::block3D_vectorno& c_alloc_blocks) {
  creation_result res;

  ER_DEBUG("Creating caches: min_dist=%f,min_blocks=%u,free_blocks=[%s] (%zu)",
           mc_min_dist.v(),
           mc_min_blocks,
           rcppsw::to_string(c_alloc_blocks).c_str(),
           c_alloc_blocks.size());

  cds::block3D_vectorno used_blocks;
  for (size_t i = 0; i < c_alloc_blocks.size() - 1; ++i) {
    cds::block3D_vectorno cache_i_blocks =
        cache_i_blocks_alloc(used_blocks, c_alloc_blocks, i);

    /*
     * We now have all the blocks that are close enough to block i to be
     * included in a new cache, so attempt cache creation.
     */
    if (cache_i_blocks.size() < mc_min_blocks) {
      continue;
    }

    if (cache_i_create(c_params,
                       c_alloc_blocks,
                       used_blocks,
                       &cache_i_blocks,
                       &res.created)) {
      /* Need to make sure we don't use these blocks in any other caches */
      used_blocks.insert(used_blocks.end(),
                         cache_i_blocks.begin(),
                         cache_i_blocks.end());
      ER_DEBUG("Used blocks after creating cache%zu: [%s]",
               i,
               rcppsw::to_string(used_blocks).c_str());
    } else {
      ++res.n_discarded;
    }
  } /* for(i..) */
  return res;
} /* create_all() */

cads::acache_vectorno dynamic_cache_creator::avoidance_caches_calc(
    const cads::acache_vectorno& c_previous_caches,
    const cads::acache_vectoro& c_created_caches) const {
  cads::acache_vectorno avoid = c_previous_caches;
  for (auto& c : c_created_caches) {
    avoid.push_back(c.get());
  } /* for(&c..) */
  return avoid;
} /* avoidance_caches_calc() */

cds::block3D_vectorno dynamic_cache_creator::absorb_blocks_calc(
    const cds::block3D_vectorno& c_alloc_blocks,
    const cds::block3D_vectorno& c_cache_i_blocks,
    const cds::block3D_vectorno& c_used_blocks,
    const rmath::vector2d& c_center,
    const rtypes::spatial_dist& c_cache_dim) const {
  cds::block3D_vectorno absorb_blocks;
  rmath::vector2d cache_dim(c_cache_dim.v(), c_cache_dim.v());

  std::copy_if(c_alloc_blocks.begin(),
               c_alloc_blocks.end(),
               std::back_inserter(absorb_blocks),
               [&](crepr::base_block3D* b) RCSW_PURE {
                 auto status = cfutils::placement_conflict2D(
                     c_center - cache_dim / 2.0,
                     cache_dim,
                     b);

                 /* block is not already used */
                 return c_used_blocks.end() == std::find(c_used_blocks.begin(),
                                                         c_used_blocks.end(),
                                                         b) &&
                     /* block is not already allocated to cache i */
                     c_cache_i_blocks.end() ==
                     std::find(c_cache_i_blocks.begin(),
                               c_cache_i_blocks.end(),
                               b) &&
                     /* block overlaps cache i */
                     status.x_conflict && status.y_conflict;
               });
  return absorb_blocks;
} /* absorb_blocks_calc() */

cds::block3D_vectorno dynamic_cache_creator::cache_i_blocks_alloc(
    const cds::block3D_vectorno& c_used_blocks,
    const cds::block3D_vectorno& c_alloc_blocks,
    size_t index) const {
  cds::block3D_vectorno cache_i_blocks;

  /*
   * Block already in a new cache, so bail out.
   */
  if (std::find(c_used_blocks.begin(),
                c_used_blocks.end(),
                c_alloc_blocks[index]) != c_used_blocks.end()) {
    return cache_i_blocks;
  }
  /*
   * Add our anchor/target block to the list of blocks for the new cache. This
   * is OK to do even if there are no other blocks close enough to create a new
   * cache, because we have a minimum # blocks threshold that has to be met
   * anyway.
   */
  ER_TRACE("Add anchor block%d@%s/%s to src list",
           c_alloc_blocks[index]->id().v(),
           rcppsw::to_string(c_alloc_blocks[index]->ranchor2D()).c_str(),
           rcppsw::to_string(c_alloc_blocks[index]->danchor2D()).c_str());
  cache_i_blocks.push_back(c_alloc_blocks[index]);
  for (size_t i = index + 1; i < c_alloc_blocks.size(); ++i) {
    /*
     * We have to check if the block is actually in the arena, because for some
     * simulations the # blocks is to great to be distributed successfull (and
     * not treated as an error), so there will be some blocks that are not
     * ACTUALLY in the arena.
     */
    if (c_alloc_blocks[index]->is_out_of_sight()) {
      continue;
    }

    /*
     * If we find a block that is close enough to our anchor/target block, then
     * add to the src list.
     */
    if (mc_min_dist >= (c_alloc_blocks[index]->rcenter2D() -
                        c_alloc_blocks[i]->rcenter2D()).length()) {
      ER_ASSERT(std::find(cache_i_blocks.begin(),
                          cache_i_blocks.end(),
                          c_alloc_blocks[i]) == cache_i_blocks.end(),
                "Block%d already on src list",
                c_alloc_blocks[i]->id().v());
      if (std::find(c_used_blocks.begin(),
                    c_used_blocks.end(),
                    c_alloc_blocks[i]) == c_used_blocks.end()) {
        ER_TRACE("Add block%d@%s/%s to src list",
                 c_alloc_blocks[i]->id().v(),
                 rcppsw::to_string(c_alloc_blocks[i]->ranchor2D()).c_str(),
                 rcppsw::to_string(c_alloc_blocks[i]->danchor2D()).c_str());
        cache_i_blocks.push_back(c_alloc_blocks[i]);
      }
    }
  } /* for(i..) */
  return cache_i_blocks;
} /* cache_i_blocks_alloc() */

bool dynamic_cache_creator::cache_i_create(
    const cache_create_ro_params& c_params,
    const cds::block3D_vectorno& c_alloc_blocks,
    const cds::block3D_vectorno& c_used_blocks,
    cds::block3D_vectorno* cache_i_blocks,
    cads::acache_vectoro* created) {

  cads::acache_vectorno c_avoid =
      avoidance_caches_calc(c_params.current_caches, *created);

  /* First, try find a conflict free cache center (host cell) */
  auto calculator = cache_center_calculator(grid(),
                                            cache_dim(),
                                            m_map->nests(),
                                            c_params.clusters);
  auto center = calculator(*cache_i_blocks, c_avoid, m_rng);
  if (!center) {
    return false;
  }

  /*
   * If we found a conflict free cache center, we need to check if there
   * are free blocks that are now within the extent of the cache to be due
   * to the center moving. If so, we absorb them into the block list for
   * the new cache.
   */
  auto absorb_blocks = absorb_blocks_calc(c_alloc_blocks,
                                          *cache_i_blocks,
                                          c_used_blocks,
                                          *center,
                                          cache_dim());
  ER_DEBUG("Absorb blocks=[%s]", rcppsw::to_string(absorb_blocks).c_str());

  cache_i_blocks->insert(cache_i_blocks->end(),
                        absorb_blocks.begin(),
                        absorb_blocks.end());
  auto cache = create_single_cache(*center, *cache_i_blocks, c_params.t);

  /*
   * If sanity checks fail, we know the problem must lie with the cache we just
   * created, because all previously created caches have already been verified.
   */
  cads::acache_vectorro sanity_caches;
  std::transform(created->begin(),
                 created->end(),
                 std::back_inserter(sanity_caches),
                 [&](const auto& c) {
                   return c.get();
                 });
  sanity_caches.push_back(cache.get());
  auto free_blocks = utils::free_blocks_calc(sanity_caches, c_alloc_blocks);

  if (!creation_sanity_checks(sanity_caches,
                              free_blocks,
                              c_params.clusters,
                              m_map->nests())) {
    if (mc_strict_constraints) {
      ER_WARN("Bad cache%d@%s/%s creation--discard (strict constraints)",
              cache->id().v(),
              rcppsw::to_string(cache->rcenter2D()).c_str(),
              rcppsw::to_string(cache->dcenter2D()).c_str());
      cache_delete(*cache_i_blocks, std::move(cache));
      return false;
    } else {
      ER_WARN("Bad cache%d@%s/%s creation--keep (loose constraints)",
              cache->id().v(),
              rcppsw::to_string(cache->rcenter2D()).c_str(),
              rcppsw::to_string(cache->dcenter2D()).c_str());
      created->push_back(std::shared_ptr<carepr::arena_cache>(std::move(cache)));

      return true;
    }
  } else {
    ER_INFO("Cache%d@%s/%s creation sanity checks OK",
            cache->id().v(),
            rcppsw::to_string(cache->rcenter2D()).c_str(),
            rcppsw::to_string(cache->dcenter2D()).c_str());
    created->push_back(std::shared_ptr<carepr::arena_cache>(std::move(cache)));
    return true;
  }
} /* cache_i_create() */

void dynamic_cache_creator::cache_delete(
    const cds::block3D_vectorno& cache_i_blocks,
    std::unique_ptr<carepr::arena_cache> victim) {
  auto xspan = victim->xdspan();
  auto yspan = victim->ydspan();

  /* clear out all cache host cell AND cells that were in CACHE_EXTENT */
  for (size_t i = xspan.lb(); i <= xspan.ub() ; ++i) {
    for (size_t j = yspan.lb(); j <= yspan.ub(); ++j) {
      auto dcoord = rmath::vector2z(i, j);
      auto& cell = m_map->access<cds::arena_grid::kCell>(dcoord);
      events::cell2D_empty_visitor op(dcoord);
      op.visit(cell);
    } /* for(j..) */
  }   /* for(i..) */

  /* redistribute blocks */
  for (auto *b : cache_i_blocks) {
    /*
     * We are actually holding zero locks, but we don't need to take any
     * since dynamic cache creation always happens in a non-concurrent
     * context.
     */
    m_map->distribute_single_block(b,
                                   carena::arena_map_locking::ekALL_HELD);
  } /* for(*b..) */
} /* cache_delete() */

NS_END(depth2, support, fordyca);
