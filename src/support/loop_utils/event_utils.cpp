/**
 * @file event_utils.cpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/controller/base_controller.hpp"
#include "fordyca/ds/arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, utils);

/*******************************************************************************
 * Functions
 ******************************************************************************/
int robot_on_block(const controller::base_controller& controller,
                   const ds::arena_map& map) {
  return map.robot_on_block(controller.position2D());
} /* robot_on_block() */

int robot_id(const controller::base_controller& controller) {
  /* +2 because the ID string starts with 'fb' */
  return std::atoi(controller.GetId().c_str() + 2);
} /* robot_id() */

int robot_on_cache(const controller::base_controller& controller,
                   const ds::arena_map& map) {
  return map.robot_on_cache(controller.position2D());
} /* robot_on_cache() */

bool block_drop_overlap_with_cache(
    const std::shared_ptr<repr::base_block>& block,
    const std::shared_ptr<repr::arena_cache>& cache,
    const rmath::vector2d& drop_loc) {
  auto drop_xspan = repr::base_entity::xspan(drop_loc, block->dims().x());
  auto drop_yspan = repr::base_entity::yspan(drop_loc, block->dims().y());
  return cache->xspan().overlaps_with(drop_xspan) &&
      cache->yspan().overlaps_with(drop_yspan);
} /* block_drop_overlap_with_cache() */

bool block_drop_near_arena_boundary(
    const ds::arena_map& map,
    const std::shared_ptr<repr::base_block>& block,
    const rmath::vector2d& drop_loc) {
  return (drop_loc.x() <= block->xdimr() * 2 ||
          drop_loc.x() >= map.xrsize() - block->xdimr() * 2 ||
          drop_loc.y() <= block->ydimr() * 2 ||
          drop_loc.y() >= map.yrsize() - block->ydimr() * 2);
} /* block_drop_overlap_with_nest() */

bool block_drop_overlap_with_nest(
    const std::shared_ptr<repr::base_block>& block,
    const repr::nest& nest,
    const rmath::vector2d& drop_loc) {
  auto drop_xspan = repr::base_entity::xspan(drop_loc, block->dims().x());
  auto drop_yspan = repr::base_entity::yspan(drop_loc, block->dims().y());

  return nest.xspan().overlaps_with(drop_xspan) &&
      nest.yspan().overlaps_with(drop_yspan);
} /* block_drop_overlap_with_nest() */

proximity_status_t cache_site_block_proximity(const controller::base_controller& c,
                                              const ds::arena_map& map,
                                              rtypes::spatial_dist block_prox) {
  for (const auto& b : map.blocks()) {
    if (block_prox >= (b->rloc() - c.position2D()).length()) {
      return {b->id(), b->rloc(), b->rloc() - c.position2D()};
    }
  } /* for(&b..) */
  return {-1, rmath::vector2d(), rmath::vector2d()};
} /* cache_site_block_proximity() */

proximity_status_t new_cache_cache_proximity(
    const controller::base_controller& c,
    const ds::arena_map& map,
    rtypes::spatial_dist new_cache_prox) {
  for (const auto& cache : map.caches()) {
    if (new_cache_prox >= (cache->rloc() - c.position2D()).length()) {
      return {cache->id(), cache->rloc(), cache->rloc() - c.position2D()};
    }
  } /* for(&b..) */
  return {-1, rmath::vector2d(), rmath::vector2d()};
} /* new_cache_cache_proximity() */


NS_END(utils, support, fordyca);
