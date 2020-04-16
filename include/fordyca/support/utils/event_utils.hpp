/**
 * \file event_utils.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_UTILS_EVENT_UTILS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_UTILS_EVENT_UTILS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include <argos3/core/simulator/entity/floor_entity.h>

#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/spatial_dist.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "fordyca/controller/controller_fwd.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {
class entity2D;
class nest;
} /* namespace cosm::repr */
namespace cosm::arena {
class caching_arena_map;
namespace repr {
class arena_cache;
} /* namespace repr */
} /* namespace cosm::arena */
namespace fordyca::repr {
class foraging_los;
}

NS_START(fordyca, support, utils);

/*******************************************************************************
 * Types
 ******************************************************************************/
struct proximity_status_t {
  rtypes::type_uuid entity_id{rtypes::constants::kNoUUID};
  rmath::vector2d entity_loc{};
  rmath::vector2d distance{};
};

/*******************************************************************************
 * Free Functions
 ******************************************************************************/
/**
 * \brief Check if a robot is on top of a block. If, so return the block index.
 *
 * \param controller The robot to check.
 * \param map \ref arena_map reference.
 *
 * \note Holding the arena map block mutex necessary in multi-threaded contexts;
 *       this is *NOT* handled internally by this function.
 *
 * \return The block index, or \ref rtypes::constants::kNoUUID if the robot is
 * not on top of a block.
 */
rtypes::type_uuid robot_on_block(const controller::foraging_controller& controller,
                                 const carena::caching_arena_map& map) RCSW_PURE;

/**
 * \brief Check if a robot is on top of a cache. If, so return the cache index.
 *
 * \param controller The robot to check.
 * \param map \ref arena_map reference.
 *
 * \note Holding the arena map cache mutex necessary in multi-threaded contexts;
 *       this is *NOT* handled internally by this function.
 *
 * \return The cache index, or \ref rtypes::constants::kNoUUID if the robot is
 * not on top of a cache.
 */
rtypes::type_uuid robot_on_cache(const controller::foraging_controller& controller,
                                 const carena::caching_arena_map& map) RCSW_PURE;

/**
 * \brief Determine if creating a new cache centered at the robot's current
 * position will overlap with any other caches in the arena/be too close to
 * them. This is an approximate check, because the weighted centroid of
 * constituent blocks is used rather than the robot's current location when
 * creating a new cache, but this should serve as a good check against invalid
 * cache creation.
 *
 * \note Holding the arena map cache mutex necessary in multi-threaded contexts;
 *       this is *NOT* handled internally by this function.
 *
 * \return (cache id of cache that is too close (-1 if none), distance to said
 *         cache).
 */
proximity_status_t new_cache_cache_proximity(const controller::foraging_controller& c,
                                             const carena::caching_arena_map& map,
                                             rtypes::spatial_dist new_cache_prox);


NS_END(utils, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_UTILS_EVENT_UTILS_HPP_ */
