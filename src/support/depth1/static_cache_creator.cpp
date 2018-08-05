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
static_cache_creator::static_cache_creator(
    const std::shared_ptr<rcppsw::er::server>& server,
    representation::arena_grid& grid,
    const argos::CVector2& center,
    double cache_size,
    double resolution)
    : cache_creator(server, grid, cache_size, resolution), m_center(center) {
  client::insmod("static_cache_creator",
                 rcppsw::er::er_lvl::DIAG,
                 rcppsw::er::er_lvl::NOM);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
cache_creator::cache_vector static_cache_creator::create_all(
    block_vector& blocks) {
  std::vector<std::shared_ptr<representation::arena_cache>> caches;

  ER_ASSERT(blocks.size() >= base_cache::kMinBlocks,
            "FATAL: Cannot create static cache from <= %u blocks",
            base_cache::kMinBlocks);
  ER_NOM("Creating static cache @(%f, %f) from %zu free blocks",
         m_center.GetX(),
         m_center.GetY(),
         blocks.size());
  block_list starter_blocks;
  for (auto b : blocks) {
    starter_blocks.push_back(b);
  } /* for(i..) */

  auto cache = cache_creator::create_single(starter_blocks, m_center);
  auto cache_p = std::shared_ptr<representation::arena_cache>(std::move(cache));
  caches.push_back(cache_p);
  return caches;
} /* create() */

NS_END(depth1, support, fordyca);
