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

#include "cosm/arena/base_arena_map.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, utils);

/*******************************************************************************
 * Functions
 ******************************************************************************/
cds::block3D_vectorno free_blocks_calc(const cads::acache_vectorro& all_caches,
                                       const cds::block3D_vectorno& all_blocks) {
  cds::block3D_vectorno free_blocks;
  std::copy_if(all_blocks.begin(),
               all_blocks.end(),
               std::back_inserter(free_blocks),
               [&](const auto& b) RCSW_PURE {
                 /* block not carried by robot */
                 return rtypes::constants::kNoUUID == b->md()->robot_id() &&
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
