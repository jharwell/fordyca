/**
 * \file event_utils.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/support/utils/event_utils.hpp"

#include "cosm/foraging/ds/arena_map.hpp"

#include "fordyca/controller/base_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, utils);

/*******************************************************************************
 * Functions
 ******************************************************************************/
rtypes::type_uuid robot_on_block(const controller::base_controller& controller,
                                 const cfds::arena_map& map) {
  return map.robot_on_block(controller.position2D());
} /* robot_on_block() */

rtypes::type_uuid robot_on_cache(const controller::base_controller& controller,
                                 const cfds::arena_map& map) {
  return map.robot_on_cache(controller.position2D());
} /* robot_on_cache() */

bool block_drop_overlap_with_cache(
    const crepr::base_block2D* const block,
    const std::shared_ptr<cfrepr::arena_cache>& cache,
    const rmath::vector2d& drop_loc) {
  auto drop_xspan = crepr::entity2D::xspan(drop_loc, block->dims().x());
  auto drop_yspan = crepr::entity2D::yspan(drop_loc, block->dims().y());
  return cache->xspan().overlaps_with(drop_xspan) &&
         cache->yspan().overlaps_with(drop_yspan);
} /* block_drop_overlap_with_cache() */

bool block_drop_near_arena_boundary(const cfds::arena_map& map,
                                    const crepr::base_block2D* const block,
                                    const rmath::vector2d& drop_loc) {
  return (drop_loc.x() <= block->xdimr() * 2 ||
          drop_loc.x() >= map.xrsize() - block->xdimr() * 2 ||
          drop_loc.y() <= block->ydimr() * 2 ||
          drop_loc.y() >= map.yrsize() - block->ydimr() * 2);
} /* block_drop_overlap_with_nest() */

bool block_drop_overlap_with_nest(const crepr::base_block2D* const block,
                                  const crepr::nest& nest,
                                  const rmath::vector2d& drop_loc) {
  auto drop_xspan = crepr::entity2D::xspan(drop_loc, block->dims().x());
  auto drop_yspan = crepr::entity2D::yspan(drop_loc, block->dims().y());

  return nest.xspan().overlaps_with(drop_xspan) &&
         nest.yspan().overlaps_with(drop_yspan);
} /* block_drop_overlap_with_nest() */

proximity_status_t new_cache_cache_proximity(
    const controller::base_controller& c,
    const cfds::arena_map& map,
    rtypes::spatial_dist new_cache_prox) {
  std::scoped_lock lock(map.cache_mtx());
  for (const auto& cache : map.caches()) {
    if (new_cache_prox >= (cache->rloc() - c.position2D()).length()) {
      return {cache->id(), cache->rloc(), cache->rloc() - c.position2D()};
    }
  } /* for(&b..) */
  return {rtypes::constants::kNoUUID, rmath::vector2d(), rmath::vector2d()};
} /* new_cache_cache_proximity() */

void handle_arena_free_block_drop(cfevents::arena_block_drop_visitor& drop_op,
                                  cfds::arena_map& map,
                                  bool drop_conflict) {
  map.block_mtx().lock();
  map.grid_mtx().lock();

  if (!drop_conflict) {
    drop_op.visit(map);
  } else {
    auto block = drop_op.block();
    map.distribute_single_block(block);
  }
  map.grid_mtx().unlock();
  map.block_mtx().unlock();
} /* handle_arena_free_block_drop() */

bool free_block_drop_conflict(const cfds::arena_map& map,
                              const crepr::base_block2D* const block,
                              const rmath::vector2d& loc) {
  /*
     * If the robot is currently right on the edge of a cache, we can't just
     * drop the block here, as it wipll overlap with the cache, and robots
     * will think that is accessible, but will not be able to vector to it
     * (not all 4 wheel sensors will report the color of a block). See #233.
     */
  bool conflict = false;
  map.cache_mtx().lock();
  for (auto& cache : map.caches()) {
    if (utils::block_drop_overlap_with_cache(block, cache, loc)) {
      conflict = true;
    }
  } /* for(cache..) */
  map.cache_mtx().unlock();

  /*
     * If the robot is currently right on the edge of the nest, we can't just
     * drop the block in the nest, as it will not be processed as a normal
     * \ref block_nest_drop, and will be discoverable by a robot via LOS but
     * not able to be acquired, as its color is hidden by that of the nest.
     *
     * If the robot is really close to a wall, then dropping a block may make
     * it inaccessible to future robots trying to reach it, due to obstacle
     * avoidance kicking in. This can result in an endless loop if said block
     * is the only one a robot knows about (see #242).
     */
  if (utils::block_drop_overlap_with_nest(block, map.nest(), loc) ||
      utils::block_drop_near_arena_boundary(map, block, loc)) {
    conflict = true;
  }
  return conflict;
} /* free_block_drop_conflict() */

NS_END(utils, support, fordyca);
