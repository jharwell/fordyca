/**
 * @file perceived_arena_map.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
 *
 * This file is part of RCPPSW.
 *
 * RCPPSW is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * RCPPSW is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * RCPPSW.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/params/grid_params.hpp"
#include "fordyca/events/block_found.hpp"
#include "fordyca/events/cache_found.hpp"
#include "fordyca/events/cell_empty.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
perceived_arena_map::perceived_arena_map(
    const std::shared_ptr<rcppsw::common::er_server>& server,
    const struct params::perceived_grid_params* params,
    const std::string& robot_id) :
    m_server(server), m_grid(&params->grid) {
  deferred_init(m_server);
  insmod("perceived_arena_map",
         rcppsw::common::er_lvl::DIAG,
         rcppsw::common::er_lvl::NOM);
  server_handle()->dbglvl(rcppsw::common::er_lvl::NOM);
  ER_NOM("%zu x %zu @ %f resolution", m_grid.xsize(), m_grid.ysize(),
         m_grid.resolution());

  for (size_t i = 0; i < m_grid.xsize(); ++i) {
    for (size_t j = 0; j < m_grid.ysize(); ++j) {
      perceived_cell2D& cell = m_grid.access(i, j);
      cell.rho(params->pheromone_rho);
      cell.robot_id(robot_id);
    } /* for(j..) */
  } /* for(i..) */
}

/*******************************************************************************
 * Events
 ******************************************************************************/
bool perceived_arena_map::event_new_los(const line_of_sight* los) {
  bool rval = false;
  for (size_t x = 0; x < los->sizex(); ++x) {
    for (size_t y = 0; y < los->sizey(); ++y) {
      representation::discrete_coord abs = los->cell(x, y).loc();
      if (los->cell(x, y).state_has_block()) {
        rval = true;
        block* block = const_cast<representation::block*>(los->cell(x,
                                                                    y).block());
        ER_ASSERT(block, "ERROR: NULL block on cell that should have block");
        if (!m_grid.access(abs.first, abs.second).state_has_block()) {
          ER_NOM("Discovered block%d at (%zu, %zu)", block->id(), abs.first,
                 abs.second);
        }

        events::block_found op(m_server, block);
        m_grid.access(abs.first, abs.second).accept(op);
      } else if (los->cell(x, y).state_has_cache()) {
        rval = true;
        cache* cache = const_cast<representation::cache*>(los->cell(x,
                                                                    y).cache());
        ER_ASSERT(cache, "ERROR: NULL cache on cell that should have cache");
        if (!m_grid.access(abs.first, abs.second).state_has_cache()) {
          ER_NOM("Discovered cache%d at (%zu, %zu)", cache->id(), abs.first,
                 abs.second);
        }

        events::cache_found op(m_server, cache);
        m_grid.access(abs.first, abs.second).accept(op);
      } else { /* must be empty if it doesn't have a block or a cache */
        events::cell_empty op;
        m_grid.access(abs.first, abs.second).accept(op);
      }
    } /* for(y..) */
  } /* for(x..) */
  return rval;
} /* event_new_los() */

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

void perceived_arena_map::update_density(void) {
  for (size_t i = 0; i < m_grid.xsize(); ++i) {
    for (size_t j = 0; j < m_grid.ysize(); ++j) {
      m_grid.access(i, j).update_density();
    } /* for(j..) */
  } /* for(i..) */
} /* update_density() */

NS_END(representation, fordyca);
