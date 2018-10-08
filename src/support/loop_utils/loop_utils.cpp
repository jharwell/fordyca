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
  return map.robot_on_block(controller.robot_loc());
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
  return map.robot_on_cache(controller.robot_loc());
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
    const argos::CVector2& drop_loc) {
  return cache->xspan(cache->real_loc()).overlaps_with(block->xspan(drop_loc)) &&
         cache->yspan(cache->real_loc()).overlaps_with(block->yspan(drop_loc));
} /* block_drop_overlap_with_cache() */

__rcsw_pure bool block_drop_near_arena_boundary(
    const ds::arena_map& map,
    const std::shared_ptr<representation::base_block>& block,
    const argos::CVector2& drop_loc) {
  return (drop_loc.GetX() <= block->xsize() * 2 ||
          drop_loc.GetX() >= map.xrsize() - block->xsize() * 2 ||
          drop_loc.GetY() <= block->ysize() * 2 ||
          drop_loc.GetY() >= map.yrsize() - block->ysize() * 2);
} /* block_drop_overlap_with_nest() */

__rcsw_pure bool block_drop_overlap_with_nest(
    const std::shared_ptr<representation::base_block>& block,
    const representation::nest& nest,
    const argos::CVector2& drop_loc) {
  return nest.xspan(nest.real_loc()).overlaps_with(block->xspan(drop_loc)) &&
         nest.yspan(nest.real_loc()).overlaps_with(block->yspan(drop_loc));
} /* block_drop_overlap_with_nest() */

std::pair<int, argos::CVector2> cache_site_block_proximity(
    const controller::base_controller& c,
    const ds::arena_map& map) {
  for (size_t j = 0; j < map.blocks().size(); ++j) {
    auto new_xspan = map.caches()[j]->xspan(c.robot_loc());
    auto new_yspan = map.caches()[j]->yspan(c.robot_loc());
    auto c_xspan = map.caches()[j]->xspan(map.caches()[j]->real_loc());
    auto c_yspan = map.caches()[j]->yspan(map.caches()[j]->real_loc());
    if (new_xspan.overlaps_with(c_xspan) || new_yspan.overlaps_with(c_yspan)) {
      return std::make_pair(j, map.caches()[j]->real_loc() - c.robot_loc());
    }
  } /* for(j..) */
  return std::make_pair(-1, argos::CVector2());
} /* cache_site_block_proximity() */
std::pair<int, argos::CVector2> new_cache_cache_proximity(
    const controller::base_controller& c,
    const ds::arena_map& map,
    double proximity_dist) {
  for (const auto& b : map.blocks()) {
    if ((b->real_loc() - c.robot_loc()).Length() >= proximity_dist) {
      return std::make_pair(b->id(), b->real_loc() - c.robot_loc());
    }
  } /* for(&b..) */
  return std::make_pair(-1, argos::CVector2());
} /* new_cache_cache_proximity() */

NS_END(loop_utils, support, fordyca);
