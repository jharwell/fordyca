/**
 * @file perceived_arena_map.cpp
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
#include "fordyca/representation/perceived_arena_map.hpp"

#include "fordyca/events/cell_empty.hpp"
#include "fordyca/params/depth0/occupancy_grid_params.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "rcppsw/er/server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);
using representation::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
perceived_arena_map::perceived_arena_map(
    std::shared_ptr<rcppsw::er::server> server,
    const struct fordyca::params::depth0::occupancy_grid_params* c_params,
    const std::string& robot_id)
    : m_server(std::move(server)),
      m_grid(m_server, c_params, robot_id),
      m_caches(),
      m_blocks() {
  deferred_client_init(m_server);
  insmod("perceived_arena_map",
         rcppsw::er::er_lvl::DIAG,
         rcppsw::er::er_lvl::NOM);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<perceived_block> perceived_arena_map::perceived_blocks(
    void) const {
  std::list<perceived_block> pblocks;

  for (auto& b : m_blocks) {
    pblocks.push_back(representation::perceived_block(b.get(),
                                                      m_grid.access<0>(b->discrete_loc())));
  } /* for(&b..) */
  return pblocks;
} /* blocks() */

std::list<perceived_cache> perceived_arena_map::perceived_caches(void) const {
  std::list<perceived_cache> pcaches;

  for (auto& c : m_caches) {
    pcaches.push_back(representation::perceived_cache(
        c.get(),
        m_grid.access<occupancy_grid::kPheromoneLayer>(c->discrete_loc())));
  } /* for(c..) */
  return pcaches;
} /* caches() */

void perceived_arena_map::cache_add(
    std::unique_ptr<base_cache>& cache) {
  cache_remove(cache.get());
  m_caches.push_back(std::move(cache));
} /* cache_add() */

void perceived_arena_map::cache_remove(const base_cache* victim) {
  for (auto it = m_caches.begin(); it != m_caches.end(); ++it) {
    if (*(*it) == *victim) {
      events::cell_empty op(victim->discrete_loc().first,
                            victim->discrete_loc().second);
      m_grid.access<occupancy_grid::kCellLayer>(victim->discrete_loc()).accept(op);
      m_caches.erase(it);
      return;
    }
  } /* for(it..) */
} /* cache_remove() */

bool perceived_arena_map::block_add(
    std::unique_ptr<block>& block) {
  auto it1 =
      std::find_if(m_blocks.begin(),
                   m_blocks.end(),
                   [&block](const std::unique_ptr<representation::block>& b) {
                     return b->id() == block->id();
                   });
  auto it2 =
      std::find_if(m_blocks.begin(),
                   m_blocks.end(),
                   [&block](const std::unique_ptr<representation::block>& b) {
                     return b->discrete_loc() == block->discrete_loc();
                   });

  if (m_blocks.end() != it1) { /* block is known */
    /*
     * Unless a given block's location has changed, there is no need to update
     * the state of the world.
     */
    if (block->discrete_loc() != (*it1)->discrete_loc()) {
      ER_VER("block%d has moved: (%zu, %zu) -> (%zu, %zu)",
             block->id(),
             (*it1)->discrete_loc().first,
             (*it1)->discrete_loc().second,
             block->discrete_loc().first,
             block->discrete_loc().second);
      int id = block->id();
      block_remove((*it1).get());
      m_blocks.push_back(std::move(block));
      ER_VER("Add block%d (n_blocks=%zu)", id, m_blocks.size());
      return true;
    }
  } else { /* block is not known */
    /*
     * A different block is currently tracked where the new block was seen, and
     * so the old block needs to be removed, as it is out of date information
     * about the arena.
     */
    if (it2 != m_blocks.end()) {
      ER_VER("Remove old block%d@(%zu, %zu): new block%d found there",
             (*it2)->id(),
             block->discrete_loc().first,
             block->discrete_loc().second,
             block->id());
      block_remove((*it2).get());
    }
    int id = block->id();
    m_blocks.push_back(std::move(block));
    ER_VER("Add block%d (n_blocks=%zu)", id, m_blocks.size());
    return true;
  }
  return false;
} /* block_add() */

bool perceived_arena_map::block_remove(const block* victim) {
  for (auto it = m_blocks.begin(); it != m_blocks.end(); ++it) {
    if (*(*it) == *victim) {
      ER_VER("Remove block%d", victim->id());
      events::cell_empty op(victim->discrete_loc().first,
                            victim->discrete_loc().second);
      m_grid.access<occupancy_grid::kCellLayer>(victim->discrete_loc())
          .accept(op);
      m_blocks.erase(it);
      return true;
    }
  } /* for(it..) */
  return false;
} /* block_remove() */

NS_END(representation, fordyca);
