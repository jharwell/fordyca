/**
 * @file dynamic_cache_manager.cpp
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
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/depth2/dynamic_cache_manager.hpp"
#include "fordyca/ds/arena_grid.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/math/utils.hpp"
#include "fordyca/params/arena/cache_params.hpp"
#include "fordyca/representation/arena_cache.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/support/depth2/dynamic_cache_creator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);
using ds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dynamic_cache_manager::dynamic_cache_manager(
    const struct params::arena::cache_params* params,
    ds::arena_grid* const arena_grid)
    : base_cache_manager(arena_grid),
      ER_CLIENT_INIT("fordyca.support.depth2.dynamic_cache_manager"),
      mc_cache_params(*params) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::pair<bool, dynamic_cache_manager::cache_vector> dynamic_cache_manager::create(
    cache_vector& existing_caches,
    block_vector& blocks) {

  support::depth2::dynamic_cache_creator creator(arena_grid(),
                                                 mc_cache_params.dimension,
                                                 mc_cache_params.min_dist);

  auto created = creator.create_all(existing_caches, blocks);
  /*
   * Must be after fixing hidden blocks, otherwise the cache host cell will
   * have a block as its entity!
   */
  creator.update_host_cells(created);
  return std::make_pair(true, created);
} /* dynamic_cache_create() */

NS_END(depth2, support, fordyca);
