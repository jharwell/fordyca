/**
 * @file event_utils.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_UTILS_EVENT_UTILS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_UTILS_EVENT_UTILS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/math/vector2.hpp"
#include "fordyca/nsalias.hpp"
#include "rcppsw/types/spatial_dist.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);

namespace controller {
class base_controller;
}
namespace repr {
class arena_cache;
class nest;
class base_entity;
class line_of_sight;
class base_block;
}
namespace ds { class arena_map; }

NS_START(support, utils);

/*******************************************************************************
 * Types
 ******************************************************************************/
struct proximity_status_t {
  int entity_id{-1};
  rmath::vector2d entity_loc{};
  rmath::vector2d distance{};
};


/*******************************************************************************
 * Free Functions
 ******************************************************************************/
/**
 * @brief Return the integer representation of the robot's unique identifier in
 * the simulation.
 */
int robot_id(const controller::base_controller& controller);

/**
 * @brief Check if a robot is on top of a block. If, so return the block index.
 *
 * @param robot The robot to check.
 * @param map \ref arena_map reference.
 *
 * @return The block index, or -1 if the robot is not on top of a block.
 */
int robot_on_block(const controller::base_controller& controller,
                   const ds::arena_map& map) RCSW_PURE;

/**
 * @brief Check if a robot is on top of a cache. If, so return the cache index.
 *
 * @param robot The robot to check.
 * @param map \ref arena_map reference.
 *
 * @return The cache index, or -1 if the robot is not on top of a cache.
 */
int robot_on_cache(const controller::base_controller& controller,
                   const ds::arena_map& map) RCSW_PURE;

bool block_drop_overlap_with_cache(
    const std::shared_ptr<repr::base_block>& block,
    const std::shared_ptr<repr::arena_cache>& cache,
    const rmath::vector2d& drop_loc) RCSW_CONST;

bool block_drop_near_arena_boundary(
    const ds::arena_map& map,
    const std::shared_ptr<repr::base_block>& block,
    const rmath::vector2d& drop_loc) RCSW_PURE;

bool block_drop_overlap_with_nest(
    const std::shared_ptr<repr::base_block>& block,
    const repr::nest& nest,
    const rmath::vector2d& drop_loc) RCSW_PURE;

/**
 * @brief Determine if creating dropping a block at the robot's current
 * position will cause a cache to be created because it is too close to other
 * blocks in the arena (blocks that are unknown to the robot).
 *
 * @return (block id of cache that is too close (-1 if none), distance to said
 *         block).
 */
proximity_status_t cache_site_block_proximity(const controller::base_controller& c,
                                              const ds::arena_map& map,
                                              rtypes::spatial_dist block_prox);

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
proximity_status_t new_cache_cache_proximity(const controller::base_controller& c,
                                             const ds::arena_map& map,
                                             rtypes::spatial_dist new_cache_prox);

NS_END(utils, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_UTILS_EVENT_UTILS_HPP_ */
