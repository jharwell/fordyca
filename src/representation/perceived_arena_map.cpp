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
#include "fordyca/representation/base_cache.hpp"
#include "fordyca/representation/block.hpp"
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
perceived_arena_map::perceived_block_list perceived_arena_map::perceived_blocks(
    void) const {
  perceived_block_list pblocks;

  for (auto& b : m_blocks) {
    pblocks.push_back(perceived_block(
        b, m_grid.access<occupancy_grid::kPheromoneLayer>(b->discrete_loc())));
  } /* for(&b..) */
  return pblocks;
} /* blocks() */

perceived_arena_map::perceived_cache_list perceived_arena_map::perceived_caches(
    void) const {
  perceived_cache_list pcaches;

  for (auto& c : m_caches) {
    pcaches.push_back(perceived_cache(
        c, m_grid.access<occupancy_grid::kPheromoneLayer>(c->discrete_loc())));
  } /* for(c..) */
  return pcaches;
} /* caches() */

void perceived_arena_map::cache_add(const std::shared_ptr<base_cache>& cache) {
  cache_remove(cache);
  m_caches.push_back(cache);
} /* cache_add() */

void perceived_arena_map::cache_remove(const std::shared_ptr<base_cache>& victim) {
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

bool perceived_arena_map::block_add(const std::shared_ptr<block>& block_in) {
  auto it1 =
      std::find_if(m_blocks.begin(),
                   m_blocks.end(),
                   [&block_in](const std::shared_ptr<representation::block>& b) {
                     return b->id() == block_in->id();
                   });
  auto it2 =
      std::find_if(m_blocks.begin(),
                   m_blocks.end(),
                   [&block_in](const std::shared_ptr<representation::block>& b) {
                     return b->discrete_loc() == block_in->discrete_loc();
                   });

  if (m_blocks.end() != it1) { /* block is known */
    /*
     * Unless a given block's location has changed, there is no need to update
     * the state of the world.
     */
    if (block_in->discrete_loc() != (*it1)->discrete_loc()) {
      ER_VER("block%d has moved: (%zu, %zu) -> (%zu, %zu)",
             block_in->id(),
             (*it1)->discrete_loc().first,
             (*it1)->discrete_loc().second,
             block_in->discrete_loc().first,
             block_in->discrete_loc().second);
      int id = block_in->id();
      block_remove(*it1);
      m_blocks.push_back(block_in);
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
             block_in->discrete_loc().first,
             block_in->discrete_loc().second,
             block_in->id());
      block_remove(*it2);
    }
    m_blocks.push_back(block_in);
    ER_VER("Add block%d (n_blocks=%zu)", block_in->id(), m_blocks.size());
    return true;
  }
  return false;
} /* block_add() */

bool perceived_arena_map::block_remove(const std::shared_ptr<block>& victim) {
  for (auto it = m_blocks.begin(); it != m_blocks.end(); ++it) {
    if (*(*it) == *victim) {
      ER_VER("Remove block%d", victim->id());
      events::cell_empty op(victim->discrete_loc().first,
                            victim->discrete_loc().second);
      m_grid.access<occupancy_grid::kCellLayer>(victim->discrete_loc()).accept(op);
      m_blocks.erase(it);
      return true;
    }
  } /* for(it..) */
  return false;
} /* block_remove() */

NS_END(representation, fordyca);
