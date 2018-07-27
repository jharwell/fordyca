/**
 * @file loop_functions_utils.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/support/loop_functions_utils.hpp"
#include "fordyca/controller/base_foraging_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, utils);

/*******************************************************************************
 * Functions
 ******************************************************************************/
__rcsw_pure int robot_on_block(
    const controller::base_foraging_controller& controller,
    const representation::arena_map& map) {
  return map.robot_on_block(controller.robot_loc());
} /* robot_on_block() */

__rcsw_pure int robot_on_block(argos::CFootBotEntity& robot,
                               const representation::arena_map& map) {
  return robot_on_block(dynamic_cast<controller::base_foraging_controller&>(
                            robot.GetControllableEntity().GetController()),
                        map);
} /* robot_on_block() */

int robot_id(argos::CFootBotEntity& robot) {
  return robot_id(dynamic_cast<controller::base_foraging_controller&>(
      robot.GetControllableEntity().GetController()));
} /* robot_id() */

int robot_id(const controller::base_foraging_controller& controller) {
  /* +2 because the ID string starts with 'fb' */
  return std::atoi(controller.GetId().c_str() + 2);
} /* robot_id() */

__rcsw_pure int robot_on_cache(
    const controller::base_foraging_controller& controller,
    const representation::arena_map& map) {
  return map.robot_on_cache(controller.robot_loc());
} /* robot_on_cache() */

__rcsw_pure int robot_on_cache(argos::CFootBotEntity& robot,
                               const representation::arena_map& map) {
  return robot_on_cache(dynamic_cast<controller::base_foraging_controller&>(
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
    const representation::arena_map& map,
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

NS_END(utils, support, fordyca);
