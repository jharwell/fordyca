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
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/params/arena_map_params.hpp"
#include "fordyca/support/cache_creator.hpp"
#include "fordyca/support/cache_update_handler.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
arena_map::arena_map(const struct params::arena_map_params* params,
                     argos::CRange<argos::Real> nest_x,
                     argos::CRange<argos::Real> nest_y) :
    mc_cache_params(params->cache),
    m_blocks(params->block.n_blocks,
             block(params->block.dimension)),
    m_caches(),
    m_block_distributor(argos::CRange<argos::Real>(params->grid.lower.GetX(),
                                                   params->grid.upper.GetX()),
                        argos::CRange<argos::Real>(params->grid.lower.GetY(),
                                                   params->grid.upper.GetY()),
                        nest_x, nest_y,
                        &params->block),
    m_server(rcppsw::common::g_server),
    m_grid(&params->grid, m_server) {
  deferred_init(m_server);
  insmod("arena_map",
         rcppsw::common::er_lvl::DIAG,
         rcppsw::common::er_lvl::NOM);

  ER_NOM("%zu x %zu @ %f resolution", m_grid.xsize(), m_grid.ysize(),
         m_grid.resolution());
  for (size_t i = 0; i < m_grid.xsize(); ++i) {
    for (size_t j = 0; j < m_grid.ysize(); ++j) {
      cell2D& cell = m_grid.access(i, j);
      cell.loc(discrete_coord(i, j));
    } /* for(j..) */
  } /* for(i..) */

  for (size_t i = 0; i < m_blocks.size(); ++i) {
    m_blocks[i].id(static_cast<int>(i));
  } /* for(i..) */
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
int arena_map::robot_on_block(const argos::CVector2& pos) {
  for (size_t i = 0; i < m_blocks.size(); ++i) {
    if (m_blocks[i].contains_point(pos)) {
      return static_cast<int>(i);
    }
  } /* for(i..) */
  return -1;
} /* robot_on_block() */

void arena_map::distribute_block(block* const block, bool first_time) {
  cell2D* cell = nullptr;
  while (1) {
    argos::CVector2 r_coord;
    if (m_block_distributor.distribute_block(*block, first_time, &r_coord)) {
      discrete_coord d_coord = representation::real_to_discrete_coord(r_coord,
                                                                      m_grid.resolution());
      cell = &m_grid.access(d_coord.first, d_coord.second);

      /*
       * You can only distribute blocks to cells that do not currently have
       * anything in them.
       */
      if (!cell->state_has_block() && !cell->state_has_cache()) {
        break;
      }
    } else {
      break;
    }
  } /* while() */
  events::free_block_drop op(m_server, block, m_grid.resolution());
  cell->accept(op);
  ER_NOM("Block%d: real_loc=(%f, %f) discrete_loc=(%zu, %zu) ptr=%p",
         block->id(),
         block->real_loc().GetX(),
         block->real_loc().GetY(),
         block->discrete_loc().first,
         block->discrete_loc().second, static_cast<const void*>(cell->block()));

  if (!first_time && mc_cache_params.create_caches) {
    support::cache_update_handler c(m_server, m_caches);
  }
} /* distribute_block() */

void arena_map::distribute_blocks(bool first_time) {
  for (size_t i = 0; i < m_blocks.size(); ++i) {
    argos::CVector2 real_coord;
    distribute_block(&m_blocks[i], first_time);
  } /* for(i..) */

  if (first_time && mc_cache_params.create_caches) {
    support::cache_creator c(m_server, m_grid, m_blocks,
                             mc_cache_params.min_dist,
                             mc_cache_params.dimension, m_grid.resolution());
    m_caches = c.create_all();
  }

  /*
   * Once all blocks have been distributed, and (possibly) all caches have been
   * created via block consolidation, then all cells that do not have blocks or
   * caches are empty.
   */
  for (size_t i = 0; i < m_grid.xsize(); ++i) {
    for (size_t j = 0; j < m_grid.ysize(); ++j) {
      cell2D& cell = m_grid.access(i, j);
      if (!cell.state_has_block() && !cell.state_has_cache()) {
        events::cell_empty op;
        cell.accept(op);
      }
    } /* for(j..) */
  } /* for(i..) */
} /* distribute_blocks() */

NS_END(representation, fordyca);
