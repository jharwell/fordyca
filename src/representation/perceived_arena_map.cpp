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

#include "fordyca/params/depth0/perceived_arena_map_params.hpp"
#include "fordyca/representation/cache.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "rcppsw/er/server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
perceived_arena_map::perceived_arena_map(
    std::shared_ptr<rcppsw::er::server> server,
    const struct params::depth0::perceived_arena_map_params *c_params,
    const std::string& robot_id)
    : m_server(std::move(server)),
      m_grid(c_params->grid.resolution,
             static_cast<size_t>(c_params->grid.upper.GetX()),
             static_cast<size_t>(c_params->grid.upper.GetY()),
             m_server),
      m_caches(),
      m_blocks() {
  deferred_init(m_server);
  insmod("perceived_arena_map",
         rcppsw::er::er_lvl::DIAG,
         rcppsw::er::er_lvl::NOM);
  ER_NOM("%zu x %zu @ %f resolution",
         m_grid.xsize(),
         m_grid.ysize(),
         m_grid.resolution());

  for (size_t i = 0; i < m_grid.xsize(); ++i) {
    for (size_t j = 0; j < m_grid.ysize(); ++j) {
      perceived_cell2D &cell = m_grid.access(i, j);
      cell.pheromone_rho(c_params->pheromone.rho);
      cell.pheromone_repeat_deposit(c_params->pheromone.repeat_deposit);
      cell.robot_id(robot_id);
      cell.decoratee().loc(discrete_coord(i, j));
    } /* for(j..) */
  }   /* for(i..) */
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<const_perceived_block> perceived_arena_map::perceived_blocks(void) const {
  std::list<const_perceived_block> pblocks;

  for (auto &b : m_blocks) {
    representation::const_perceived_block p(b.get(),
                                            m_grid
                                            .access(b->discrete_loc().first,
                                                    b->discrete_loc().second)
                                            .density());
    pblocks.push_back(p);
  } /* for(&b..) */
  return pblocks;
} /* blocks() */

std::list<const_perceived_cache> perceived_arena_map::perceived_caches(void) const {
  std::list<const_perceived_cache> pcaches;

  for (auto &c : m_caches) {
    representation::const_perceived_cache p(c.get(),
                                            m_grid
                                            .access(c->discrete_loc().first,
                                                    c->discrete_loc().second)
                                            .density());
    pcaches.push_back(p);
  } /* for(c..) */
  return pcaches;
} /* caches() */

void perceived_arena_map::update_density(void) {
  for (size_t i = 0; i < m_grid.xsize(); ++i) {
    for (size_t j = 0; j < m_grid.ysize(); ++j) {
      m_grid.access(i, j).density_update();
    } /* for(j..) */
  }   /* for(i..) */
} /* update_density() */

void perceived_arena_map::cache_add(
    std::unique_ptr<representation::cache>& cache) {
  cache_remove(cache.get());
  m_caches.push_back(std::move(cache));
} /* cache_add() */

void perceived_arena_map::cache_remove(const cache* victim) {
  for (auto it = m_caches.begin(); it != m_caches.end(); ++it) {
    if (*(*it) == *victim) {
      events::cell_empty op(victim->discrete_loc().first,
                            victim->discrete_loc().second);
      m_grid.access(victim->discrete_loc().first,
                    victim->discrete_loc().second).accept(op);
      m_caches.erase(it);
      return;
    }
  } /* for(it..) */
} /* cache_remove() */

bool perceived_arena_map::block_add(
    std::unique_ptr<representation::block>& block) {
  auto it1 = std::find_if(m_blocks.begin(),
                         m_blocks.end(),
                         [&block](const std::unique_ptr<representation::block>& b) {
                           return b->id() == block->id(); });
  auto it2 = std::find_if(m_blocks.begin(),
                          m_blocks.end(),
                          [&block](const std::unique_ptr<representation::block>& b) {
                            return b->discrete_loc() == block->discrete_loc(); });

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
      m_grid.access(victim->discrete_loc().first,
                    victim->discrete_loc().second).accept(op);
      m_blocks.erase(it);
      return true;
    }
  } /* for(it..) */
  return false;
} /* block_remove() */

NS_END(representation, fordyca);
