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
    representation::const_perceived_block p(&b,
                                            m_grid
                                            .access(b.discrete_loc().first,
                                                    b.discrete_loc().second)
                                            .density());
    pblocks.push_back(p);
  } /* for(&b..) */
  return pblocks;
} /* blocks() */

std::list<const_perceived_cache> perceived_arena_map::perceived_caches(void) const {
  std::list<const_perceived_cache> pcaches;

  for (auto &c : m_caches) {
    representation::const_perceived_cache p(&c,
                                            m_grid
                                            .access(c.discrete_loc().first,
                                                    c.discrete_loc().second)
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

void perceived_arena_map::cache_add(representation::cache &cache) {
  auto it = std::find(m_caches.begin(), m_caches.end(), cache);
  if (m_caches.end() != it) {
    cache_remove(*it);
  }
  m_caches.push_back(cache);
} /* cache_add() */

void perceived_arena_map::cache_remove(representation::cache &victim) {
  m_caches.erase(std::remove(m_caches.begin(), m_caches.end(), victim));
} /* cache_remove() */

void perceived_arena_map::block_add(representation::block &block) {
  auto it = std::find(m_blocks.begin(), m_blocks.end(), block);
  /*
   * If we already know about the block that we are adding from our current LOS,
   * remove the old version iff its location has changed (i.e. has been carried
   * to the nest and the re-distributed) since we last saw it, and add an
   * updated version of the new block. If the location of a previously known
   * block has not changed then don't do anything, as the state of a block can't
   * change other than to appear/disappear in perceived map.
   *
   * If we don't currently know about the block, then add it (obviously).
   */
  if (m_blocks.end() != it) {
    if (block.discrete_loc() != it->discrete_loc()) {
      block_remove(*it);
      events::cell_empty op((*it).discrete_loc().first,
                            (*it).discrete_loc().second);
      m_grid.access((*it).discrete_loc().first,
                    (*it).discrete_loc().second).accept(op);
    m_blocks.push_back(block);
    }
  } else {
    m_blocks.push_back(block);
  }
} /* block_add() */

void perceived_arena_map::block_remove(representation::block &victim) {
  /*
   * @bug For some reason when I erase from this vector and the victim element
   * has already been erased and the vector no longer contains it, I get a
   * segmentation fault. Not sure why. This fixes it, at least for now. See
   * #226, #227, #228.
   */
  if (!m_blocks.empty()) {
    m_blocks.erase(std::remove(m_blocks.begin(), m_blocks.end(), victim));
  }
} /* block_remove() */

NS_END(representation, fordyca);
