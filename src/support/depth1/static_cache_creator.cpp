/**
 * @file static_cache_creator.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/depth1/static_cache_creator.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/representation/arena_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);
using representation::base_cache;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
static_cache_creator::static_cache_creator(ds::arena_grid* const grid,
                                           const rmath::vector2d& center,
                                           double cache_size)
    : base_cache_creator(grid, cache_size),
      ER_CLIENT_INIT("fordyca.support.depth1.static_cache_creator"),
      m_center(center) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
ds::cache_vector static_cache_creator::create_all(
    const ds::cache_vector& existing_caches,
    ds::block_vector& blocks,
    double) {
  ER_ASSERT(existing_caches.empty(), "Static cache already exists in arena!");
  ds::cache_vector caches;

  ER_ASSERT(blocks.size() >= base_cache::kMinBlocks,
            "Cannot create static cache from < %zu blocks",
            base_cache::kMinBlocks);
  ER_INFO("Creating static cache@%s from %zu free blocks",
          m_center.to_str().c_str(),
          blocks.size());
  ds::block_list starter_blocks;
  for (auto& b : blocks) {
    starter_blocks.push_back(b);
  } /* for(i..) */

  auto cache = create_single_cache(starter_blocks, m_center);
  auto cache_p = std::shared_ptr<representation::arena_cache>(std::move(cache));
  caches.push_back(cache_p);
  return caches;
} /* create() */

NS_END(depth1, support, fordyca);
