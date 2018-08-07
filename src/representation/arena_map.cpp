/**
 * @file arena_map.cpp
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
#include "fordyca/representation/arena_map.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/params/arena/arena_map_params.hpp"
#include "fordyca/representation/arena_cache.hpp"
#include "fordyca/representation/block_manifest_processor.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "fordyca/representation/cube_block.hpp"
#include "fordyca/representation/ramp_block.hpp"
#include "fordyca/support/depth1/static_cache_creator.hpp"
#include "rcppsw/er/server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
arena_map::arena_map(const struct params::arena::arena_map_params* params)
    : client(rcppsw::er::g_server),
      mc_static_cache_params(params->static_cache),
      m_blocks(block_manifest_processor(&params->blocks.dist.manifest)
                   .create_blocks()),
      m_caches(),
      m_grid(params->grid.resolution,
             static_cast<size_t>(params->grid.upper.GetX()),
             static_cast<size_t>(params->grid.upper.GetY()),
             server_ref()),
      m_nest(params->nest.dims, params->nest.center, params->grid.resolution),
      m_block_dispatcher(rcppsw::er::g_server, m_grid, &params->blocks.dist) {
  insmod("arena_map", rcppsw::er::er_lvl::DIAG, rcppsw::er::er_lvl::NOM);

  ER_NOM("%zu x %zu/%zu x %zu @ %f resolution",
         m_grid.xdsize(),
         m_grid.ydsize(),
         m_grid.xrsize(),
         m_grid.yrsize(),
         m_grid.resolution());
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool arena_map::initialize(void) {
  return m_block_dispatcher.initialize();
} /* initialize() */

__rcsw_pure int arena_map::robot_on_block(const argos::CVector2& pos) const {
  for (size_t i = 0; i < m_blocks.size(); ++i) {
    if (m_blocks[i]->contains_point(pos)) {
      return static_cast<int>(i);
    }
  } /* for(i..) */
  return -1;
} /* robot_on_block() */

__rcsw_pure int arena_map::robot_on_cache(const argos::CVector2& pos) const {
  for (size_t i = 0; i < m_caches.size(); ++i) {
    if (m_caches[i]->contains_point(pos)) {
      return static_cast<int>(i);
    }
  } /* for(i..) */
  return -1;
} /* robot_on_cache() */

void arena_map::static_cache_create(void) {
  ER_DIAG("(Re)-Creating static cache");
  argos::CVector2 center((m_grid.xdsize() + m_nest.real_loc().GetX()) / 2.0,
                         m_nest.real_loc().GetY());

  support::depth1::static_cache_creator c(server_ref(),
                                          m_grid,
                                          center,
                                          mc_static_cache_params.dimension,
                                          m_grid.resolution());

  block_vector blocks;

  /*
   * Only blocks that are not:
   *
   * - Currently carried by a robot
   * - Currently placed on the cell where the cache is to be created
   *
   * are eligible for being used to re-create the static cache.
   */
  for (auto& b : m_blocks) {
    if (-1 == b->robot_id() &&
        b->discrete_loc() !=
            math::rcoord_to_dcoord(center, m_grid.resolution())) {
      blocks.push_back(b);
    }
    if (blocks.size() >= mc_static_cache_params.size) {
      break;
    }
  } /* for(b..) */

  m_caches = c.create_all(blocks);
  c.update_host_cells(m_grid, m_caches);
} /* static_cache_create() */

bool arena_map::distribute_single_block(std::shared_ptr<base_block>& block) {
  support::block_dist::dispatcher::entity_list entities;
  for (auto& cache : m_caches) {
    entities.push_back(cache.get());
  } /* for(&cache..) */
  for (auto& b : m_blocks) {
    if (b != block) {
      entities.push_back(b.get());
    }
  } /* for(&b..) */
  entities.push_back(&m_nest);
  return m_block_dispatcher.distribute_block(block, entities);
} /* disribute_single_block() */

void arena_map::distribute_all_blocks(void) {
  // Reset all the cells to clear old references to blocks
  m_grid.reset();

  /* distribute blocks */
  support::block_dist::dispatcher::entity_list entities;
  for (auto& cache : m_caches) {
    entities.push_back(cache.get());
  } /* for(&cache..) */
  entities.push_back(&m_nest);
  ER_ASSERT(m_block_dispatcher.distribute_blocks(m_blocks, entities),
            "FATAL: Unable to perform initial block distribution");

  /*
   * Once all blocks have been distributed, and (possibly) all caches have been
   * created via block consolidation, then all cells that do not have blocks or
   * caches are empty.
   */
  for (size_t i = 0; i < m_grid.xdsize(); ++i) {
    for (size_t j = 0; j < m_grid.ydsize(); ++j) {
      cell2D& cell = m_grid.access(i, j);
      if (!cell.state_has_block() && !cell.state_has_cache()) {
        events::cell_empty op(i, j);
        cell.accept(op);
      }
    } /* for(j..) */
  }   /* for(i..) */
} /* distribute_all_blocks() */

void arena_map::cache_remove(const std::shared_ptr<arena_cache>& victim) {
  m_caches.erase(std::remove(m_caches.begin(), m_caches.end(), victim));
} /* cache_remove() */

NS_END(representation, fordyca);
