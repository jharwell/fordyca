/**
 * @file loop_utils.cpp
 *
 * @copyright 201c John Harwell, All rights reserved.
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
#include "fordyca/support/loop_utils/loop_utils.hpp"
#include "fordyca/controller/base_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, loop_utils);

/*******************************************************************************
 * Functions
 ******************************************************************************/
__rcsw_pure int robot_on_block(const controller::base_controller& controller,
                               const ds::arena_map& map) {
  return map.robot_on_block(controller.position());
} /* robot_on_block() */

__rcsw_pure int robot_on_block(argos::CFootBotEntity& robot,
                               const ds::arena_map& map) {
  return robot_on_block(dynamic_cast<controller::base_controller&>(
                            robot.GetControllableEntity().GetController()),
                        map);
} /* robot_on_block() */

int robot_id(argos::CFootBotEntity& robot) {
  return robot_id(dynamic_cast<controller::base_controller&>(
      robot.GetControllableEntity().GetController()));
} /* robot_id() */

int robot_id(const controller::base_controller& controller) {
  /* +2 because the ID string starts with 'fb' */
  return std::atoi(controller.GetId().c_str() + 2);
} /* robot_id() */

__rcsw_pure int robot_on_cache(const controller::base_controller& controller,
                               const ds::arena_map& map) {
  return map.robot_on_cache(controller.position());
} /* robot_on_cache() */

__rcsw_pure int robot_on_cache(argos::CFootBotEntity& robot,
                               const ds::arena_map& map) {
  return robot_on_cache(dynamic_cast<controller::base_controller&>(
                            robot.GetControllableEntity().GetController()),
                        map);
}

__rcsw_const bool block_drop_overlap_with_cache(
    const std::shared_ptr<representation::base_block>& block,
    const std::shared_ptr<representation::arena_cache>& cache,
    const rmath::vector2d& drop_loc) {
  return cache->xspan(cache->real_loc()).overlaps_with(block->xspan(drop_loc)) &&
         cache->yspan(cache->real_loc()).overlaps_with(block->yspan(drop_loc));
} /* block_drop_overlap_with_cache() */

__rcsw_pure bool block_drop_near_arena_boundary(
    const ds::arena_map& map,
    const std::shared_ptr<representation::base_block>& block,
    const rmath::vector2d& drop_loc) {
  return (drop_loc.x() <= block->xsize() * 2 ||
          drop_loc.x() >= map.xrsize() - block->xsize() * 2 ||
          drop_loc.y() <= block->ysize() * 2 ||
          drop_loc.y() >= map.yrsize() - block->ysize() * 2);
} /* block_drop_overlap_with_nest() */

__rcsw_pure bool block_drop_overlap_with_nest(
    const std::shared_ptr<representation::base_block>& block,
    const representation::nest& nest,
    const rmath::vector2d& drop_loc) {
  return nest.xspan(nest.real_loc()).overlaps_with(block->xspan(drop_loc)) &&
         nest.yspan(nest.real_loc()).overlaps_with(block->yspan(drop_loc));
} /* block_drop_overlap_with_nest() */

proximity_status_t cache_site_block_proximity(
    const controller::base_controller& c,
    const ds::arena_map& map,
    double block_prox_dist) {
  for (const auto& b : map.blocks()) {
    if ((b->real_loc() - c.position()).length() <= block_prox_dist) {
      return {b->id(), b->real_loc() - c.position()};
    }
  } /* for(&b..) */
  return {-1, rmath::vector2d()};
} /* cache_site_block_proximity() */

proximity_status_t new_cache_cache_proximity(
    const controller::base_controller& c,
    const ds::arena_map& map,
    double proximity_dist) {
  for (const auto& cache : map.caches()) {
    if ((cache->real_loc() - c.position()).length() <= proximity_dist) {
      return {cache->id(), cache->real_loc() - c.position()};
    }
  } /* for(&b..) */
  return {-1, rmath::vector2d()};
} /* new_cache_cache_proximity() */

placement_status_t placement_conflict(const rmath::vector2d& rloc,
                               const rmath::vector2d& dims,
                               const representation::multicell_entity* const entity) {
  auto movable =
      dynamic_cast<const representation::movable_cell_entity*>(entity);
  auto immovable =
      dynamic_cast<const representation::immovable_cell_entity*>(entity);

  auto loc_xspan = representation::multicell_entity::xspan(rloc,
                                                           dims.x());
  auto loc_yspan = representation::multicell_entity::yspan(rloc,
                                                           dims.y());
  placement_status_t status;
  if (nullptr != movable) {
    auto ent_xspan = entity->xspan(movable->real_loc());
    auto ent_yspan = entity->yspan(movable->real_loc());
    status.x_conflict = ent_xspan.overlaps_with(loc_xspan);
    status.y_conflict = ent_yspan.overlaps_with(loc_yspan);
  } else {
    auto ent_xspan = entity->xspan(immovable->real_loc());
    auto ent_yspan = entity->yspan(immovable->real_loc());
    status.x_conflict = ent_xspan.overlaps_with(loc_xspan);
    status.y_conflict = ent_yspan.overlaps_with(loc_yspan);
  }
  return status;
} /* placement_conflict() */

NS_END(loop_utils, support, fordyca);
