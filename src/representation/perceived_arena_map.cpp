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

#include "rcppsw/er/server.hpp"
#include "fordyca/params/perceived_grid_params.hpp"
#include "fordyca/representation/cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
perceived_arena_map::perceived_arena_map(
    const std::shared_ptr<rcppsw::er::server>& server,
    const struct params::perceived_grid_params* params,
    const std::string& robot_id) :
    m_server(server),
    m_grid(params->grid.resolution, params->grid.upper.GetX(),
           params->grid.upper.GetY(), m_server),
    m_caches() {
  deferred_init(m_server);
  insmod("perceived_arena_map",
         rcppsw::er::er_lvl::DIAG,
         rcppsw::er::er_lvl::NOM);
  ER_NOM("%zu x %zu @ %f resolution", m_grid.xsize(), m_grid.ysize(),
         m_grid.resolution());

  for (size_t i = 0; i < m_grid.xsize(); ++i) {
    for (size_t j = 0; j < m_grid.ysize(); ++j) {
      perceived_cell2D& cell = m_grid.access(i, j);
      cell.rho(params->pheromone_rho);
      cell.robot_id(robot_id);
      cell.cell().loc(discrete_coord(i, j));
    } /* for(j..) */
  } /* for(i..) */
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<perceived_block> perceived_arena_map::blocks(void) const {
  std::list<perceived_block> blocks;
  for (size_t i = 0; i < m_grid.xsize(); ++i) {
    for (size_t j = 0; j < m_grid.ysize(); ++j) {
      if (m_grid.access(i, j).state_has_block()) {
        blocks.push_back(perceived_block(m_grid.access(i, j).block(),
                                         m_grid.access(i, j).density()));
      }
    } /* for(j..) */
  } /* for(i..) */
  return blocks;
} /* blocks() */

std::list<perceived_cache> perceived_arena_map::perceived_caches(void) const {
  std::list<perceived_cache> pcaches;
  for (auto& c : m_caches) {
    representation::perceived_cache p(&c,
                                      m_grid.access(c.discrete_loc().first,
                                                    c.discrete_loc().second).density());
    pcaches.push_back(p);
  } /* for(c..) */

  return pcaches;
} /* caches() */

void perceived_arena_map::update_density(void) {
  for (size_t i = 0; i < m_grid.xsize(); ++i) {
    for (size_t j = 0; j < m_grid.ysize(); ++j) {
      m_grid.access(i, j).update_density();
    } /* for(j..) */
  } /* for(i..) */
} /* update_density() */

void perceived_arena_map::cache_add(representation::cache& cache) {
  m_caches.push_back(cache);
} /* cache_add() */

void perceived_arena_map::cache_remove(representation::cache& victim) {
  m_caches.erase(std::remove(m_caches.begin(),
                             m_caches.end(), victim));
} /* cache_remove() */

NS_END(representation, fordyca);
