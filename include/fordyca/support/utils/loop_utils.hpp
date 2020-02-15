/**
 * \file loop_utils.hpp
 * \ingroup support utils
 *
 * Helpers for loop functions that CAN be free functions, as they do not require
 * access to anything in \ref argos::CLoopFunctions.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_UTILS_LOOP_UTILS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_UTILS_LOOP_UTILS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/types/spatial_dist.hpp"
#include "fordyca/fordyca.hpp"
#include "fordyca/ds/block_vector.hpp"
#include "fordyca/ds/cache_vector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class entity2D;
} /* namespace cosm::repr */

NS_START(fordyca);
namespace controller {
class base_controller;
}
namespace repr {
class arena_cache;
class nest;
class line_of_sight;
class base_block2D;
}
namespace ds { class arena_map; }
NS_START(support, utils);

/*******************************************************************************
 * Types
 ******************************************************************************/
struct placement_status_t {
  bool x_conflict{false};
  bool y_conflict{false};
};

/*******************************************************************************
 * Functions
 ******************************************************************************/
/**
 * \brief Compute the line of sight for a given robot.
 *
 * Needed to eliminate header dependencies in this file.
 */
std::unique_ptr<repr::line_of_sight> compute_robot_los(
    const ds::arena_map& map,
    uint los_grid_size,
    const rmath::vector2d& pos);

/**
 * \brief Set the LOS of a robot in the arena.
 *
 * This is a hack that makes getting my research up and running easier.
 *
 * \todo This should eventually be replaced by a calculation of a robot's LOS by
 * the robot, probably using on-board cameras.
 */
template <typename T>
void set_robot_los(T* const controller,
                   uint los_grid_size,
                   ds::arena_map& map) {
  controller->los(std::move(compute_robot_los(map,
                                              los_grid_size,
                                              controller->position2D())));
}

template<typename T>
void set_robot_tick(argos::CFootBotEntity& robot, rtypes::timestep t) {
  auto& controller = dynamic_cast<T&>(robot.GetControllableEntity().GetController());
  controller.tick(t);
}

/**
 * \brief Determine if an entity of the specified dimensions, placed at the
 * specified location (or that currently exists at the specified location), will
 * overlap the specified (different) entity (or does overlap it).
 *
 */
placement_status_t placement_conflict(const rmath::vector2d& ent1_loc,
                                      const rmath::vector2d& ent1_dims,
                                      const crepr::entity2D* entity);

/**
 * \brief Calculate the blocks that are:
 *
 * - Not carried by a robot
 * - Not inside a cache
 *
 * \param all_caches All existing caches in the arena.
 * \param all_blocks All blocks in the arena.
 */
ds::block_vector free_blocks_calc(const ds::cache_vector& all_caches,
                                  const ds::block_vector& all_blocks);

NS_END(utils, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_UTILS_LOOP_UTILS_HPP_ */
