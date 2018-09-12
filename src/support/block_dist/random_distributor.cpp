/**
 * @file random_block_distributor.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/support/block_dist/random_distributor.hpp"
#include <algorithm>

#include <boost/uuid/uuid_io.hpp>
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/math/utils.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "fordyca/representation/immovable_cell_entity.hpp"
#include "fordyca/representation/multicell_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, block_dist);
namespace er = rcppsw::er;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
random_distributor::random_distributor(representation::arena_grid::view& grid,
                                       double resolution)
    : base_distributor(),
      ER_CLIENT_INIT("fordyca.support.block_dist.random"),
      m_resolution(resolution),
      m_grid(grid) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool random_distributor::distribute_blocks(block_vector& blocks,
                                           entity_list& entities) {
  ER_INFO("Distributing %zu blocks in area: xrange=[%u-%lu], yrange=[%u-%lu]",
          blocks.size(),
          (*m_grid.origin()).loc().first,
          (*m_grid.origin()).loc().first + m_grid.shape()[0],
          (*m_grid.origin()).loc().second,
          (*m_grid.origin()).loc().second + m_grid.shape()[1]);
  for (auto& b : blocks) {
    if (!distribute_block(b, entities)) {
      return false;
    }
  } /* for(&b..) */
  return true;
} /* distribute_blocks() */

bool random_distributor::distribute_block(
    std::shared_ptr<representation::base_block>& block,
    entity_list& entities) {
  representation::cell2D* cell = nullptr;
  std::vector<uint> coord;
  if (!find_avail_coord(entities, coord)) {
    return false;
  }
  ER_DEBUG("Found coordinates for distribution: rel=(%u, %u), abs=(%u, %u)",
           coord[0],
           coord[1],
           coord[2],
           coord[3]);

  cell = &m_grid[coord[0]][coord[1]];

  /*
   * You can only distribute blocks to cells that do not currently have
   * anything in them. If there is already something there, then our
   * distribution algorithm has a bug.
   */
  ER_ASSERT(!cell->state_has_block(), "Destination cell already contains block");
  ER_ASSERT(!cell->state_has_cache(), "Destination cell already contains cache");
  ER_ASSERT(!cell->state_in_cache_extent(),
            "Destination cell part of cache extent");

  events::free_block_drop op(block,
                             rcppsw::math::dcoord2(coord[2], coord[3]),
                             m_resolution);
  cell->accept(op);
  if (verify_block_dist(*block, cell)) {
    /*
       * Now that the block has been distributed, it is another entity that
       * needs to be distributed around.
       */
    entities.push_back(block.get());
    return true;
  } else {
    return false;
  }
} /* distribute_block() */

__rcsw_pure bool random_distributor::verify_block_dist(
    const representation::base_block& block,
    __rcsw_unused const representation::cell2D* const cell) {
  /* blocks should not be out of sight after distribution... */
  ER_CHECK(representation::base_block::kOutOfSightDLoc != block.discrete_loc(),
           "Block%d discrete coordinates still out of sight after "
           "distribution",
           block.id());
  ER_CHECK(representation::base_block::kOutOfSightRLoc != block.real_loc(),
           "Block%d real coordinates still out of sight after distribution",
           block.id());

  ER_INFO("Block%d: real_loc=(%f, %f) discrete_loc=(%u, %u) ptr=%p",
          block.id(),
          block.real_loc().GetX(),
          block.real_loc().GetY(),
          block.discrete_loc().first,
          block.discrete_loc().second,
          reinterpret_cast<const void*>(cell->block().get()));
  return true;

error:
  return false;
} /* verify_block_dist() */

bool random_distributor::find_avail_coord(const entity_list& entities,
                                          std::vector<uint>& coordv) {
  uint abs_x, abs_y, rel_x, rel_y;
  rcppsw::math::range<uint> area_xrange(m_grid.index_bases()[0],
                                        m_grid.shape()[0]);
  rcppsw::math::range<uint> area_yrange(m_grid.index_bases()[1],
                                        m_grid.shape()[1]);

  /* -1 because we are working with array indices */
  std::uniform_int_distribution<uint> xdist(area_xrange.get_min(),
                                            area_xrange.get_max() - 1);
  std::uniform_int_distribution<uint> ydist(area_yrange.get_min(),
                                            area_yrange.get_max() - 1);
  uint count = 0;

  do {
    rel_x = area_xrange.span() > 0 ? xdist(m_rng) : m_grid.index_bases()[0];
    rel_y = area_xrange.span() > 0 ? ydist(m_rng) : m_grid.index_bases()[1];
    abs_x = rel_x + (*m_grid.origin()).loc().first;
    abs_y = rel_y + (*m_grid.origin()).loc().second;
  } while (
      std::any_of(
          entities.begin(),
          entities.end(),
          [&](const representation::multicell_entity* ent) {
            auto movable =
                dynamic_cast<const representation::movable_cell_entity*>(ent);
            argos::CVector2 coord =
                math::dcoord_to_rcoord(rcppsw::math::dcoord2(abs_x, abs_y),
                                       m_resolution);
            if (nullptr != movable) {
              auto ent_xspan = ent->xspan(movable->real_loc());
              auto ent_yspan = ent->yspan(movable->real_loc());
              ER_TRACE(
                  "(movable entity) rcoord=(%f, %f), xspan=[%f-%f], "
                  "yspan=[%f-%f]",
                  coord.GetX(),
                  coord.GetY(),
                  ent_xspan.get_min(),
                  ent_xspan.get_max(),
                  ent_yspan.get_min(),
                  ent_yspan.get_max());
              return ent_xspan.value_within(coord.GetX()) &&
                     ent_yspan.value_within(coord.GetY());
            }
            auto immovable =
                dynamic_cast<const representation::immovable_cell_entity*>(ent);
            ER_ASSERT(nullptr != immovable,
                      "Cell entity is neither movable nor immovable");
            auto ent_xspan = ent->xspan(immovable->real_loc());
            auto ent_yspan = ent->yspan(immovable->real_loc());

            ER_TRACE(
                "(immovable entity) rcoord=(%f, %f), xspan=[%f-%f], "
                "yspan=[%f-%f]",
                coord.GetX(),
                coord.GetY(),
                ent_xspan.get_min(),
                ent_xspan.get_max(),
                ent_yspan.get_min(),
                ent_yspan.get_max());
            return ent_xspan.value_within(coord.GetX()) &&
                   ent_yspan.value_within(coord.GetY());
          }) &&
      count++ <= kMAX_DIST_TRIES);
  if (count <= kMAX_DIST_TRIES) {
    coordv = std::vector<uint>({rel_x, rel_y, abs_x, abs_y});
    return true;
  }
  return false;
} /* find_avail_coord() */

NS_END(block_dist, support, fordyca);
