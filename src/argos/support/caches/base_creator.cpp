/**
 * \file base_creator.cpp
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
#include "fordyca/argos/support/caches/base_creator.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/operations/cache_extent_set.hpp"
#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/arena/operations/free_block_pickup.hpp"
#include "cosm/arena/repr/light_type_index.hpp"
#include "cosm/ds/operations/cell2D_cache_extent.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"
#include "cosm/repr/sim_block3D.hpp"
#include "cosm/arena/free_blocks_calculator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, caches);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_creator::base_creator(carena::caching_arena_map* const map,
                                       const rtypes::spatial_dist& cache_dim)
    : ER_CLIENT_INIT("fordyca.argos.support.caches.base_creator"),
      mc_cache_dim(cache_dim),
      m_map(map) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::shared_ptr<carepr::arena_cache>
base_creator::create_single_cache(const rmath::vector2d& center,
                                        cds::block3D_vectorno&& blocks,
                                        const rtypes::timestep& t,
                                        bool pre_dist) {
  ER_ASSERT(center.is_pd(),
            "Center@%s is not positive definite",
            center.to_str().c_str());
  auto dcenter = rmath::dvec2zvec(center, m_map->grid_resolution().v());
  auto& cell = m_map->access<cads::arena_grid::kCell>(dcenter);

  /*
   * The cell that will be the location of the new cache may already contain a
   * block. If so, it should be in the list of blocks for the cache.
   */
  if (cell.state_has_block()) {
    ER_ASSERT(nullptr != cell.block3D(),
              "Cell@%s does not have block",
              rcppsw::to_string(cell.loc()).c_str());

    /*
     * If the host cell for the cache already contains a block, we don't want to
     * unconditionally add said block to the list of blocks for the new cache,
     * because if the cache was created using some function of free blocks in
     * the arena, then that block could already be in the list, and we would be
     * double adding it, which will cause problems later (dynamic cache
     * creation). However, it may also NOT be in the list of blocks to use for
     * the new cache, in which case we need to add in (static cache creation).
     */
    if (blocks.end() == std::find(blocks.begin(), blocks.end(), cell.block3D())) {
      /*
       * We use insert() instead of push_back() here so that if there was a
       * leftover block on the cell where a cache used to be that is also where
       * this cache is being created, it becomes the "front" of the cache, and
       * will be the first block picked up by a robot from the new cache. This
       * helps to ensure fairness/better statistics for the simulations.
       */
      ER_DEBUG("Add block%d from cache host cell@%s to block vector",
               cell.block3D()->id().v(),
               rcppsw::to_string(dcenter).c_str());
      blocks.insert(blocks.begin(), cell.block3D());
    }
  }

  /*
   * If blocks have not been distributed yet, nothing to do.
   */
  if (!pre_dist) {
    ER_INFO("Clearing host cells and block extents for %zu blocks",
            blocks.size());
    /*
     * We don't need to lock around the cell empty and block drop events because
     * cache creation always happens AFTER all robots have had their control
     * steps run, never DURING.
     */
    for (auto& block : blocks) {
      ER_DEBUG("Clearing block%d host cell@%s,extents: x=%s,y=%s, removing from "
               "cluster",
               block->id().v(),
               rcppsw::to_string(block->danchor2D()).c_str(),
               rcppsw::to_string(block->xdspan()).c_str(),
               rcppsw::to_string(block->ydspan()).c_str());

      auto pickup_op = caops::free_block_pickup_visitor::by_arena(block);
      /*
       * "Pickup/drop" block. This is OK to do even if the cache we create ends
       * up being bad, because in that case all the used blocks are
       * re-distributed.
       */
      pickup_op.visit(*m_map);
    } /* for(block..) */

  }
  /*
   * Loop through all blocks and deposit them in the cache host cell, which
   * will be in the HAS_CACHE state after this loop, but will not yet have a
   * cache as its entity.
   */
  for (auto& block : blocks) {
    caops::free_block_drop_visitor drop_op(block,
                                           dcenter,
                                           m_map->grid_resolution(),
                                           carena::locking::ekALL_HELD);

    drop_op.visit(cell);
    drop_op.visit(*block);
  } /* for(block..) */

  ER_DEBUG("All %zu blocks now in host cell%s",
           blocks.size(),
           rcppsw::to_string(dcenter).c_str());

  /* create the cache! */
  cds::block3D_vectorno for_cache(blocks.begin(), blocks.end());
  auto ret = std::make_shared<carepr::arena_cache>(
      carepr::arena_cache::params{
        mc_cache_dim,
            m_map->grid_resolution(),
            center,
            std::move(for_cache),
            rtypes::constants::kNoUUID },
      carepr::light_type_index()[carepr::light_type_index::kCache]);
  ret->creation_ts(t);

  /*
   * By default, caches use a vector to track the blocks they contain, which
   * makes it fast to clone() them for use in robot LOS. For caches that
   * actually exist in the arena, we need a fast way to check if a block is in a
   * cache or not, so we want a map implementation, which we explicitly enable
   * here.
   */
  ret->blocks_map_enable();
  ER_CONDW(dcenter != ret->dcenter2D(),
            "Created cache%d has bad center? %s != %s",
            ret->id().v(),
            rcppsw::to_string(ret->dcenter2D()).c_str(),
            rcppsw::to_string(dcenter).c_str());
  ER_INFO("Created cache%d@%s/%s,anchor=%s/%s,xspan=%s/%s,yspan=%s/%s with %zu "
          "blocks [%s]",
          ret->id().v(),
          rcppsw::to_string(ret->rcenter2D()).c_str(),
          rcppsw::to_string(ret->dcenter2D()).c_str(),
          rcppsw::to_string(ret->ranchor2D()).c_str(),
          rcppsw::to_string(ret->danchor2D()).c_str(),
          rcppsw::to_string(ret->xrspan()).c_str(),
          rcppsw::to_string(ret->xdspan()).c_str(),
          rcppsw::to_string(ret->yrspan()).c_str(),
          rcppsw::to_string(ret->ydspan()).c_str(),
          ret->n_blocks(),
          rcppsw::to_string(blocks).c_str());

  /* Update host cell */
  cell.entity(ret.get());
  cell.color(ret->color());

  return ret;
} /* create_single_cache() */

void base_creator::cache_extents_configure(
    const cads::acache_vectoro& caches) {
  for (const auto& cache : caches) {
    caops::cache_extent_set_visitor e(cache.get());
    e.visit(m_map->decoratee());
  } /* for(cache..) */
} /* cache_extents_configure() */

NS_END(caches, support, argos, fordyca);
