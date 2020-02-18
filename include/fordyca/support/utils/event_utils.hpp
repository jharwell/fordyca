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
#include "cosm/foraging/events/arena_block_drop.hpp"

#include "rcppsw/math/vector2.hpp"
#include "fordyca/fordyca.hpp"
#include "rcppsw/types/spatial_dist.hpp"
#include "rcppsw/types/type_uuid.hpp"
#include "cosm/foraging/ds/arena_map.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {
class base_block2D;
class entity2D;
class nest;
class arena_cache;
} /* namespace cosm::repr */

namespace fordyca::controller {
class base_controller;
}
namespace fordyca::repr {
class line_of_sight;
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
rtypes::type_uuid robot_on_block(const controller::base_controller& controller,
                                 const cfds::arena_map& map) RCSW_PURE;

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
rtypes::type_uuid robot_on_cache(const controller::base_controller& controller,
                   const cfds::arena_map& map) RCSW_PURE;

/**
 * \brief Determine if dropping the specified block at the specified location
 * will overlap with the specified cache.
 *
 * \return \c TRUE if so, \c FALSE otherwise.
 */
bool block_drop_overlap_with_cache(
    const crepr::base_block2D* block,
    const std::shared_ptr<cfrepr::arena_cache>& cache,
    const rmath::vector2d& drop_loc) RCSW_CONST;

/**
 * \brief Determine if dropping the specified block at the specified location
 * will be too close to arena boundaries.
 *
 * \return \c TRUE if so, \c FALSE otherwise.
 */
bool block_drop_near_arena_boundary(
    const cfds::arena_map& map,
    const crepr::base_block2D* block,
    const rmath::vector2d& drop_loc) RCSW_PURE;

/**
 * \brief Determine if dropping the specified block at the specified location
 * will overlap with the nest.
 *
 * \return \c TRUE if so, \c FALSE otherwise.
 */
bool block_drop_overlap_with_nest(
    const crepr::base_block2D* block,
    const crepr::nest& nest,
    const rmath::vector2d& drop_loc) RCSW_PURE;

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
proximity_status_t new_cache_cache_proximity(const controller::base_controller& c,
                                             const cfds::arena_map& map,
                                             rtypes::spatial_dist new_cache_prox);

/**
 * \brief Handle a free block drop in the arena with proper locking. The
 * controller associated with the block drop is *NOT* updated.
 *
 * \param drop_op The block drop event.
 * \param map \ref arena_map reference.
 * \param drop_conflict Does this block drop conflict with the positions of
 *                      other things in the arena? If so, if it will be
 *                      distributed instead of being dropped (i.e. the block
 *                      drop operation does not visit the map).
 */
void handle_arena_free_block_drop(cfevents::arena_block_drop_visitor& drop_op,
                                  cfds::arena_map& map,
                                  bool drop_conflict);

bool free_block_drop_conflict(const cfds::arena_map& map,
                              const crepr::base_block2D* const block,
                              const rmath::vector2d& loc);

NS_END(utils, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_UTILS_EVENT_UTILS_HPP_ */
