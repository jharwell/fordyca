/**
 * \file base_cache_manager.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
#include "fordyca/support/base_cache_manager.hpp"

#include <cmath>

#include "cosm/arena/caching_arena_map.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_cache_manager::bloctree_update(const cads::acache_vectoro& caches) {
  cads::acache_vectorro created;
  for (const auto & c : caches) {
    created.push_back(c.get());
  } /* for(&b..) */
  m_map->created_caches(created);

  for (const auto & cache : caches) {
    for (auto& pair : cache->blocks()) {
      m_map->bloctree_update(pair.second,
                             carena::arena_map_locking::ekALL_HELD);
    } /* for(*block..) */
  } /* for(&cache..) */

  m_map->created_caches_clear();
} /* bloctree_update() */

NS_END(support, fordyca);
