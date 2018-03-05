/**
 * @file loop_functions_utils.hpp
 * @ingroup support
 *
 * Helpers for loop functions that CAN be free functions, as they do not require
 * access to anything in \ref argos::CLoopFunctions.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_LOOP_FUNCTIONS_UTILS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_LOOP_FUNCTIONS_UTILS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include "fordyca/representation/arena_map.hpp"
#include "fordyca/representation/line_of_sight.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace controller {
class base_foraging_controller;
}

NS_START(support, utils);

/*******************************************************************************
 * Functions
 ******************************************************************************/
/**
 * @brief Check if a robot is on top of a block. If, so return the block index.
 *
 * @param robot The robot to check.
 *
 * @return The block index, or -1 if the robot is not on top of a block.
 */
int robot_on_block(argos::CFootBotEntity& robot,
                   const representation::arena_map& map);
int robot_on_block(const controller::base_foraging_controller& controller,
                   const representation::arena_map& map);

/**
 * @brief Check if a robot is on top of a cache. If, so return the cache index.
 *
 * @param robot The robot to check.
 *
 * @return The cache index, or -1 if the robot is not on top of a cache.
 */
int robot_on_cache(argos::CFootBotEntity& robot,
                   const representation::arena_map& map);
int robot_on_cache(const controller::base_foraging_controller& controller,
                   const representation::arena_map& map);
/**
 * @brief Get the ID of the robot as an integer.
 */
int robot_id(argos::CFootBotEntity& robot);
int robot_id(const controller::base_foraging_controller& controller);

bool block_drop_overlap_with_cache(const representation::block* block,
                                   const representation::arena_cache& cache,
                                   const argos::CVector2& drop_loc);

bool block_drop_near_arena_boundary(const representation::arena_map& map,
                                    const representation::block* block,
                                    const argos::CVector2& drop_loc);
bool block_drop_overlap_with_nest(const representation::block* block,
                                  const argos::CRange<double>& xrange,
                                  const argos::CRange<double>& yrange,
                                  const argos::CVector2& drop_loc);

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
  argos::CVector2 pos;
  pos.Set(const_cast<argos::CFootBotEntity&>(robot)
              .GetEmbodiedEntity()
              .GetOriginAnchor()
              .Position.GetX(),
          const_cast<argos::CFootBotEntity&>(robot)
              .GetEmbodiedEntity()
              .GetOriginAnchor()
              .Position.GetY());

  auto& controller =
      dynamic_cast<T&>(robot.GetControllableEntity().GetController());
  controller.robot_loc(pos);
}

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
                   representation::arena_map& map) {
  argos::CVector2 pos;
  pos.Set(const_cast<argos::CFootBotEntity&>(robot)
              .GetEmbodiedEntity()
              .GetOriginAnchor()
              .Position.GetX(),
          const_cast<argos::CFootBotEntity&>(robot)
              .GetEmbodiedEntity()
              .GetOriginAnchor()
              .Position.GetY());

  rcppsw::math::dcoord2 robot_loc =
      math::rcoord_to_dcoord(pos, map.grid_resolution());
  auto& controller =
      dynamic_cast<T&>(robot.GetControllableEntity().GetController());
  std::unique_ptr<representation::line_of_sight> new_los =
      rcppsw::make_unique<representation::line_of_sight>(
          map.subgrid(robot_loc.first, robot_loc.second, 2), robot_loc);
  controller.los(new_los);
}

NS_END(utils, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_LOOP_FUNCTIONS_UTILS_HPP_ */
