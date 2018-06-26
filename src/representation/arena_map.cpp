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
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/params/arena_map_params.hpp"
#include "fordyca/representation/arena_cache.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "fordyca/support/depth1/static_cache_creator.hpp"
#include "rcppsw/er/server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
arena_map::arena_map(const struct params::arena_map_params* params)
    : mc_static_cache_params(params->static_cache),
      m_blocks(params->block_dist.n_blocks),
      m_caches(),
      m_block_distributor(&params->block_dist),
      m_server(rcppsw::er::g_server),
      m_grid(params->grid.resolution,
             static_cast<size_t>(params->grid.upper.GetX()),
             static_cast<size_t>(params->grid.upper.GetY()),
             m_server),
      m_nest(params->nest.xdim,
             params->nest.ydim,
             params->nest.center,
             params->grid.resolution) {
  deferred_client_init(m_server);
  insmod("arena_map", rcppsw::er::er_lvl::DIAG, rcppsw::er::er_lvl::NOM);

  ER_NOM("%zu x %zu/%zu x %zu @ %f resolution",
         m_grid.xdsize(),
         m_grid.ydsize(),
         m_grid.xrsize(),
         m_grid.yrsize(),
         m_grid.resolution());
  for (size_t i = 0; i < m_grid.xdsize(); ++i) {
    for (size_t j = 0; j < m_grid.ydsize(); ++j) {
      cell2D& cell = m_grid.access(i, j);
      cell.loc(rcppsw::math::dcoord2(i, j));
    } /* for(j..) */
  }   /* for(i..) */

  for (size_t i = 0; i < m_blocks.size(); ++i) {
    m_blocks[i] =
        std::make_shared<block>(params->block.dimension, static_cast<int>(i));
  } /* for(i..) */
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
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

void arena_map::distribute_block(const std::shared_ptr<block>& block) {
  cell2D* cell = nullptr;
  while (true) {
    argos::CVector2 r_coord;
    if (m_block_distributor.distribute_block(*block, &r_coord)) {
      rcppsw::math::dcoord2 d_coord =
          math::rcoord_to_dcoord(r_coord, m_grid.resolution());
      cell = &m_grid.access(d_coord.first, d_coord.second);
      /*
       * You can only distribute blocks to cells that do not currently have
       * anything in them.
       */
      if (!cell->state_has_block() && !cell->state_has_cache()) {
        events::free_block_drop op(
            m_server, block, d_coord.first, d_coord.second, m_grid.resolution());
        cell->accept(op);
        break;
      }
    } else { /* no distributing needs to be done (respawn is disabled) */
      return;
    }
  } /* while() */
  ER_NOM("Block%d: real_loc=(%f, %f) discrete_loc=(%zu, %zu) ptr=%p",
         block->id(),
         block->real_loc().GetX(),
         block->real_loc().GetY(),
         block->discrete_loc().first,
         block->discrete_loc().second,
         reinterpret_cast<void*>(cell->block().get()));
} /* distribute_block() */

void arena_map::static_cache_create(void) {
  double src_center =
      (m_block_distributor.single_src_xrange(m_blocks[0]->xsize()).GetMin() +
       m_block_distributor.single_src_xrange(m_blocks[0]->xsize()).GetMax()) /
      2.0;
  double x = (src_center + m_nest.real_loc().GetX()) / 2.0;
  double y = m_nest.real_loc().GetY();

  ER_DIAG("(Re)-Creating static cache");
  support::depth1::static_cache_creator c(m_server,
                                          m_grid,
                                          argos::CVector2(x, y),
                                          mc_static_cache_params.dimension,
                                          m_grid.resolution());

  std::vector<std::shared_ptr<representation::block>> blocks;

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
        b->discrete_loc() != math::rcoord_to_dcoord(argos::CVector2(x, y),
                                                    m_grid.resolution())) {
      blocks.push_back(b);
    }
    if (blocks.size() >= mc_static_cache_params.size) {
      break;
    }
  } /* for(b..) */

  m_caches = c.create_all(blocks);
  c.update_host_cells(m_grid, m_caches);
} /* static_cache_create() */

void arena_map::distribute_blocks(void) {
  // Reset all the cells to clear old references to blocks
  m_grid.reset();

  for (auto& b : m_blocks) {
    distribute_block(b);
  } /* for(b..) */
  for (auto& b : m_blocks) {
    ER_ASSERT(representation::block::kOutOfSightDLoc != b->discrete_loc(),
              "FATAL: Block%d discrete coordinates still out of sight after "
              "distribution",
              b->id());
    ER_ASSERT(
        representation::block::kOutOfSightRLoc != b->real_loc(),
        "FATAL: Block%d real coordinates still out of sight after distribution",
        b->id());
  } /* for(b..) */

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
} /* distribute_blocks() */

void arena_map::cache_remove(const std::shared_ptr<arena_cache>& victim) {
  m_caches.erase(std::remove(m_caches.begin(), m_caches.end(), victim));
} /* cache_remove() */

NS_END(representation, fordyca);
