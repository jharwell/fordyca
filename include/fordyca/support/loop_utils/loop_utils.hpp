/**
 * @file loop_utils.hpp
 * @ingroup support loop_utils
 *
 * Helpers for loop functions that CAN be free functions, as they do not require
 * access to anything in \ref argos::CLoopFunctions.
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_LOOP_UTILS_LOOP_UTILS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_LOOP_UTILS_LOOP_UTILS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <utility>
#include "fordyca/repr/line_of_sight.hpp"
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace controller {
class base_controller;
}
namespace repr {
class arena_cache;
class nest;
class multicell_entity;
class line_of_sight;
class base_block;
}
namespace ds { class arena_map; }
NS_START(support, loop_utils);
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Types
 ******************************************************************************/
struct proximity_status_t {
  int entity_id;
  rmath::vector2d entity_loc;
  rmath::vector2d distance;
};

struct placement_status_t {
  bool x_conflict;
  bool y_conflict;
};

/*******************************************************************************
 * Functions
 ******************************************************************************/
/**
 * @brief Check if a robot is on top of a block. If, so return the block index.
 *
 * @param robot The robot to check.
 * @param map \ref arena_map reference.
 *
 * @return The block index, or -1 if the robot is not on top of a block.
 */
int robot_on_block(argos::CFootBotEntity& robot, const ds::arena_map& map);
int robot_on_block(const controller::base_controller& controller,
                   const ds::arena_map& map);

/**
 * @brief Check if a robot is on top of a cache. If, so return the cache index.
 *
 * @param robot The robot to check.
 * @param map \ref arena_map reference.
 *
 * @return The cache index, or -1 if the robot is not on top of a cache.
 */
int robot_on_cache(argos::CFootBotEntity& robot, const ds::arena_map& map);
int robot_on_cache(const controller::base_controller& controller,
                   const ds::arena_map& map);
/**
 * @brief Get the ID of the robot as an integer.
 */
int robot_id(argos::CFootBotEntity& robot);
int robot_id(const controller::base_controller& controller);

bool block_drop_overlap_with_cache(
    const std::shared_ptr<repr::base_block>& block,
    const std::shared_ptr<repr::arena_cache>& cache,
    const rmath::vector2d& drop_loc);

bool block_drop_near_arena_boundary(
    const ds::arena_map& map,
    const std::shared_ptr<repr::base_block>& block,
    const rmath::vector2d& drop_loc);
bool block_drop_overlap_with_nest(
    const std::shared_ptr<repr::base_block>& block,
    const repr::nest& nest,
    const rmath::vector2d& drop_loc);

/**
 * @brief Set the position of the robot in the arena.
 *
 * This is a hack that makes getting my research up and running easier.
 *
 * @todo This should eventually be replaced by a calculation of robot's position
 * by the robot.
 */
template <typename T>
void set_robot_pos(argos::CFootBotEntity& robot) {
  rmath::vector2d pos;
  pos.set(const_cast<argos::CFootBotEntity&>(robot)
              .GetEmbodiedEntity()
              .GetOriginAnchor()
              .Position.GetX(),
          const_cast<argos::CFootBotEntity&>(robot)
              .GetEmbodiedEntity()
              .GetOriginAnchor()
              .Position.GetY());

  auto& controller =
      dynamic_cast<T&>(robot.GetControllableEntity().GetController());
  controller.position(pos);
}

/**
 * @brief Determine if creating dropping a block at the robot's current
 * position will cause a cache to be created because it is too close to other
 * blocks in the arena (blocks that are unknown to the robot).
 *
 * @return (block id of cache that is too close (-1 if none), distance to said
 *         block).
 */
proximity_status_t cache_site_block_proximity(
    const controller::base_controller& controller,
    const ds::arena_map& map,
    double block_prox_dist);

/**
 * @brief Determine if creating a new cache centered at the robot's current
 * position will overlap with any other caches in the arena/be too close to
 * them. This is an approximate check, because the weighted centroid of
 * constituent blocks is used rather than the robot's current location when
 * creating a new cache, but this should serve as a good check against invalid
 * cache creation.
 *
 * @return (cache id of cache that is too close (-1 if none), distance to said
 *         cache).
 */
proximity_status_t new_cache_cache_proximity(
    const controller::base_controller& controller,
    const ds::arena_map& map,
    double proximity_dist);


/**
 * @brief Compute the line of sight for a given robot.
 *
 * Needed to eliminate header dependencies in this file.
 */
std::unique_ptr<repr::line_of_sight> compute_robot_los(
    ds::arena_map& map,
    uint los_grid_size,
    const rmath::vector2d& pos);

/**
 * @brief Set the LOS of a robot in the arena.
 *
 * This is a hack that makes getting my research up and running easier.
 *
 * @todo This should eventually be replaced by a calculation of a robot's LOS by
 * the robot, probably using on-board cameras.
 */
template <typename T>
void set_robot_los(argos::CFootBotEntity& robot,
                   uint los_grid_size,
                   ds::arena_map& map) {
  rmath::vector2d pos;
  pos.set(const_cast<argos::CFootBotEntity&>(robot)
              .GetEmbodiedEntity()
              .GetOriginAnchor()
              .Position.GetX(),
          const_cast<argos::CFootBotEntity&>(robot)
              .GetEmbodiedEntity()
              .GetOriginAnchor()
              .Position.GetY());

  auto& controller =
      dynamic_cast<T&>(robot.GetControllableEntity().GetController());
  controller.los(std::move(compute_robot_los(map, los_grid_size, pos)));
}


/**
 * @brief Determine if an entity of the specified dimensions, placed at the
 * specified location (or that currently exists at the specified location), will
 * overlap the specified (different) entity (or does overlap it).
 *
 */
placement_status_t placement_conflict(const rmath::vector2d& ent1_loc,
                                      const rmath::vector2d& ent1_dims,
                                      const repr::multicell_entity* const ent2);

NS_END(loop_utils, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_LOOP_UTILS_LOOP_UTILS_HPP_ */
