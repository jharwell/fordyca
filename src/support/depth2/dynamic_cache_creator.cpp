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
cads::acache_vectoro dynamic_cache_creator::create_all(
    const cache_create_ro_params& c_params,
    const cds::block3D_vectorno& c_alloc_blocks) {
  cads::acache_vectoro created_caches;

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
    cache_i_create(c_params,
                   c_alloc_blocks,
                   std::move(cache_i_blocks),
                   &created_caches,
                   &used_blocks);

    /*
     * Need to make sure we don't use these blocks in any other caches, even if
     * we aborted created cache i and they still are available, for simplicity's
     * sake.
    */
    used_blocks.insert(used_blocks.end(),
                       cache_i_blocks.begin(),
                       cache_i_blocks.end());
    ER_DEBUG("Used blocks after creating cache%zu: [%s]",
             i,
             rcppsw::to_string(used_blocks).c_str());
  } /* for(i..) */
  return created_caches;
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
    const rmath::vector2z& c_center,
    rtypes::spatial_dist cache_dim) const {
  cds::block3D_vectorno absorb_blocks;
  std::copy_if(c_alloc_blocks.begin(),
               c_alloc_blocks.end(),
               std::back_inserter(absorb_blocks),
               [&](crepr::base_block3D* b) RCSW_PURE {
                 auto xspan = rmath::ranged(c_center.x() - cache_dim.v() / 2.0,
                                            c_center.x() + cache_dim.v() / 2.0);
                 auto yspan = rmath::ranged(c_center.y() - cache_dim.v() / 2.0,
                                            c_center.y() + cache_dim.v() / 2.0);
                 return c_used_blocks.end() == std::find(c_used_blocks.begin(),
                                                         c_used_blocks.end(),
                                                         b) &&
                        c_cache_i_blocks.end() ==
                            std::find(c_cache_i_blocks.begin(),
                                      c_cache_i_blocks.end(),
                                      b) &&
                        xspan.overlaps_with(b->xrspan()) &&
                        yspan.overlaps_with(b->yrspan());
               });
  return absorb_blocks;
} /* absorb_blocks_calc() */

cds::block3D_vectorno dynamic_cache_creator::cache_i_blocks_alloc(
    const cds::block3D_vectorno& c_used_blocks,
    const cds::block3D_vectorno& c_alloc_blocks,
    uint index) const {
  cds::block3D_vectorno src_blocks;

  /*
   * Block already in a new cache, so bail out.
   */
  if (std::find(c_used_blocks.begin(),
                c_used_blocks.end(),
                c_alloc_blocks[index]) != c_used_blocks.end()) {
    return src_blocks;
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
  src_blocks.push_back(c_alloc_blocks[index]);
  for (size_t i = index + 1; i < c_alloc_blocks.size(); ++i) {
    /*
     * If we find a block that is close enough to our anchor/target block, then
     * add to the src list.
     */
    if (mc_min_dist >= (c_alloc_blocks[index]->rcenter2D() -
                        c_alloc_blocks[i]->rcenter2D()).length()) {
      ER_ASSERT(std::find(src_blocks.begin(),
                          src_blocks.end(),
                          c_alloc_blocks[i]) == src_blocks.end(),
                "Block%d already on src list",
                c_alloc_blocks[i]->id().v());
      if (std::find(c_used_blocks.begin(),
                    c_used_blocks.end(),
                    c_alloc_blocks[i]) == c_used_blocks.end()) {
        ER_TRACE("Add block%d@%s/%s to src list",
                 c_alloc_blocks[i]->id().v(),
                 rcppsw::to_string(c_alloc_blocks[i]->ranchor2D()).c_str(),
                 rcppsw::to_string(c_alloc_blocks[i]->danchor2D()).c_str());
        src_blocks.push_back(c_alloc_blocks[i]);
      }
    }
  } /* for(i..) */
  return src_blocks;
} /* cache_i_blocks_alloc() */

bool dynamic_cache_creator::cache_i_create(
    const cache_create_ro_params& c_params,
    const cds::block3D_vectorno& c_alloc_blocks,
    cds::block3D_vectorno&& cache_i_blocks,
    cads::acache_vectoro* created_caches,
    cds::block3D_vectorno* used_blocks) {

  cads::acache_vectorno c_avoid =
      avoidance_caches_calc(c_params.current_caches, *created_caches);

  /* First, try find a conflict free cache center (host cell) */
  auto center = cache_center_calculator(grid(), cache_dim())(
      cache_i_blocks, c_avoid, c_params.clusters, m_rng);
  if (!center) {
    return false;
  }

  /*
   * If we found a conflict free cache center, we need to check if there
   * are free blocks that are now within the extent of the cache to be due
   * to the center moving. If so, we absorb them into the block list for
   * the new cache.
   */
  auto absorb_blocks = absorb_blocks_calc(
      c_alloc_blocks, cache_i_blocks, *used_blocks, *center, cache_dim());
  ER_DEBUG("Absorb blocks=[%s]", rcppsw::to_string(absorb_blocks).c_str());

  cache_i_blocks.insert(cache_i_blocks.end(),
                        absorb_blocks.begin(),
                        absorb_blocks.end());
  /*
   * We convert to discrete and then back to real coordinates so that our
   * cache's real location is always on an even multiple of the grid size,
   * which keeps asserts about cache extent from triggering right after
   * creation, which can happen otherwise.
   */
  auto cache_p = std::shared_ptr<carepr::arena_cache>(create_single_cache(
      *center, cache_i_blocks, c_params.t));
  created_caches->push_back(cache_p);

  auto free_blocks = utils::free_blocks_calc(*created_caches,
                                             c_alloc_blocks);

  if (!creation_sanity_checks(*created_caches,
                              free_blocks,
                              c_params.clusters)) {
    if (mc_strict_constraints) {
      ER_WARN("Bad cache creation--discard (strict constraints)");
      cache_delete(cache_i_blocks, created_caches);
      return false;
    } else {
      ER_WARN("Bad cache creation--keep (loose constraints)");
      return true;
    }
  } else {
    ER_INFO("Cache creation sanity checks OK");
    return true;
  }
} /* cache_i_create() */

void dynamic_cache_creator::cache_delete(
    const cds::block3D_vectorno& cache_i_blocks,
    cads::acache_vectoro* created_caches) {
  auto loc = created_caches->back()->dcenter2D();

  events::cell2D_empty_visitor op(loc);
  op.visit(m_map->access<cds::arena_grid::kCell>(loc));

  for (auto *b : cache_i_blocks) {
    /*
     * We are actually holding zero locks, but we don't need to take any
     * since dynamic cache creation always happens in a non-concurrent
     * context.
     */
    m_map->distribute_single_block(b,
                                   carena::arena_map_locking::ekALL_HELD);
  } /* for(*b..) */

  created_caches->pop_back();
} /* cache_delete() */

NS_END(depth2, support, fordyca);
