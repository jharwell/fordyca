/**
 * \file event_utils.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/utils/event_utils.hpp"

#include "cosm/foraging/ds/arena_map.hpp"

#include "fordyca/controller/foraging_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, utils);

/*******************************************************************************
 * Functions
 ******************************************************************************/
rtypes::type_uuid robot_on_block(const controller::foraging_controller& controller,
                                 const cfds::arena_map& map) {
  return map.robot_on_block(controller.pos2D(), controller.entity_acquired_id());
} /* robot_on_block() */

rtypes::type_uuid robot_on_cache(const controller::foraging_controller& controller,
                                 const cfds::arena_map& map) {
  return map.robot_on_cache(controller.pos2D(), controller.entity_acquired_id());
} /* robot_on_cache() */

proximity_status_t new_cache_cache_proximity(
    const controller::foraging_controller& c,
    const cfds::arena_map& map,
    rtypes::spatial_dist new_cache_prox) {
  std::scoped_lock lock(*map.cache_mtx());
  for (const auto* cache : map.caches()) {
    if (new_cache_prox >= (cache->rloc() - c.pos2D()).length()) {
      return {cache->id(), cache->rloc(), cache->rloc() - c.pos2D()};
    }
  } /* for(&b..) */
  return {rtypes::constants::kNoUUID, rmath::vector2d(), rmath::vector2d()};
} /* new_cache_cache_proximity() */

NS_END(utils, support, fordyca);
