/**
 * @file cache_update_handler.cpp
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
#include "fordyca/support/cache_update_handler.hpp"
#include <algorithm>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_update_handler::cache_update_handler(
    std::shared_ptr<rcppsw::common::er_server> server,
    std::vector<representation::cache>& caches) :
    er_client(server), m_caches(caches) {
  insmod("cache_update_handler",
         rcppsw::common::er_lvl::DIAG,
         rcppsw::common::er_lvl::NOM);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
__pure representation::cache* cache_update_handler::map_to_cache(
    const representation::block* const block) {
  for (size_t i = 0; i < m_caches.size(); ++i) {
    ER_ASSERT(!m_caches[i].contains_block(block),
              "FATAL: block already belongs to cache %d", m_caches[i].id());
    if (m_caches[i].block_within_boundaries(block)) {
      return &m_caches[i];
    }
  } /* for(i..) */
  return nullptr;
} /* map_to_cache() */

void cache_update_handler::block_add(representation::cache* const cache,
                                     representation::block* const block) {
  ER_ASSERT(!cache->contains_block(block),
            "FATAL: block%d already belongs to cache %d",
            block->id(), cache->id());

  cache->block_add(block);
} /* block_add() */

void cache_update_handler::block_remove(representation::cache* cache,
                                        representation::block* const block) {
  ER_ASSERT(cache->contains_block(block),
            "FATAL: block%d does not belong to cache %d",
            block->id(), cache->id());

  cache->block_remove(block);
  if (0 == cache->n_blocks()) {
    m_caches.erase(std::remove(m_caches.begin(), m_caches.end(), *cache));
  }
} /* block_remove() */

NS_END(support, fordyca);
