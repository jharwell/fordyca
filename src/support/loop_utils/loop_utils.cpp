/**
 * \file loop_utils.cpp
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
#include "fordyca/support/utils/loop_utils.hpp"

#include "cosm/foraging/ds/arena_map.hpp"

#include "fordyca/controller/base_controller.hpp"
#include "fordyca/repr/line_of_sight.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, utils);

/*******************************************************************************
 * Functions
 ******************************************************************************/
placement_status_t placement_conflict(const rmath::vector2d& ent1_loc,
                                      const rmath::vector2d& ent1_dims,
                                      const crepr::entity2D* const entity) {
  auto loc_xspan = crepr::entity2D::xspan(ent1_loc, ent1_dims.x());
  auto loc_yspan = crepr::entity2D::yspan(ent1_loc, ent1_dims.y());
  return placement_status_t{entity->xspan().overlaps_with(loc_xspan),
                            entity->yspan().overlaps_with(loc_yspan)};
} /* placement_conflict() */

std::unique_ptr<repr::line_of_sight> compute_robot_los(
    const cfds::arena_map& map,
    uint los_grid_size,
    const rmath::vector2d& pos) {
  rmath::vector2u position = rmath::dvec2uvec(pos, map.grid_resolution().v());
  return std::make_unique<repr::line_of_sight>(
      map.subgrid(position.x(), position.y(), los_grid_size), position);
} /* compute_robot_los */

cfds::block_vector2 free_blocks_calc(const cfds::cache_vector& all_caches,
                                    const cfds::block_vector2& all_blocks) {
  cfds::block_vector2 free_blocks;
  std::copy_if(all_blocks.begin(),
               all_blocks.end(),
               std::back_inserter(free_blocks),
               [&](const auto& b) RCSW_PURE {
                 /* block not carried by robot */
                 return rtypes::constants::kNoUUID == b->robot_id() &&
                        /*
                      * Block not inside cache (to catch blocks that were on the
                      * host cell for the cache, and we incorporated into it
                      * during creation).
                      */
                        std::all_of(all_caches.begin(),
                                    all_caches.end(),
                                    [&](const auto& c) {
                                      return !c->contains_block(b);
                                    });
               });
  return free_blocks;
} /* free_blocks_calc() */

NS_END(utils, support, fordyca);
