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
#include "fordyca/ds/cell2D.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/representation/immovable_cell_entity.hpp"
#include "fordyca/representation/multicell_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, block_dist);
namespace er = rcppsw::er;
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
random_distributor::random_distributor(const ds::arena_grid::view& grid,
                                       double resolution)
    : base_distributor(),
      ER_CLIENT_INIT("fordyca.support.block_dist.random"),
      m_resolution(resolution),
      m_grid(grid) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool random_distributor::distribute_blocks(ds::block_vector& blocks,
                                           ds::const_entity_list& entities) {
  rmath::vector2u loc = (*m_grid.origin()).loc();
  ER_INFO("Distributing %zu blocks in area: xrange=[%u-%lu], yrange=[%u-%lu]",
          blocks.size(),
          loc.x(),
          loc.x() + m_grid.shape()[0],
          loc.y(),
          loc.y() + m_grid.shape()[1]);

  return std::all_of(blocks.begin(),
                     blocks.end(),
                     [&](std::shared_ptr<representation::base_block>& b) {
                       return distribute_block(b, entities);
                     });
} /* distribute_blocks() */

bool random_distributor::distribute_block(
    std::shared_ptr<representation::base_block>& block,
    ds::const_entity_list& entities) {
  ds::cell2D* cell = nullptr;
  std::vector<uint> coord;
  if (!find_avail_coord(entities, coord)) {
    return false;
  }
  ER_INFO("Found coordinates for distribution: rel=(%u, %u), abs=(%u, %u)",
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
                             rmath::vector2u(coord[2], coord[3]),
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
    __rcsw_unused const ds::cell2D* const cell) {
  /* blocks should not be out of sight after distribution... */
  ER_CHECK(representation::base_block::kOutOfSightDLoc != block.discrete_loc(),
           "Block%d discrete coordinates still out of sight after "
           "distribution",
           block.id());
  ER_CHECK(representation::base_block::kOutOfSightRLoc != block.real_loc(),
           "Block%d real coordinates still out of sight after distribution",
           block.id());

  ER_DEBUG("Block%d@%s/%s ptr=%p",
           block.id(),
           block.real_loc().to_str().c_str(),
           block.discrete_loc().to_str().c_str(),
           reinterpret_cast<const void*>(cell->block().get()));
  return true;

error:
  return false;
} /* verify_block_dist() */

bool random_distributor::find_avail_coord(const ds::const_entity_list& entities,
                                          std::vector<uint>& coordv) {
  uint abs_x, abs_y, rel_x, rel_y;
  rcppsw::math::rangeu area_xrange(m_grid.index_bases()[0], m_grid.shape()[0]);
  rcppsw::math::rangeu area_yrange(m_grid.index_bases()[1], m_grid.shape()[1]);

  /* -1 because we are working with array indices */
  std::uniform_int_distribution<uint> xdist(area_xrange.lb(),
                                            area_xrange.ub() - 1);
  std::uniform_int_distribution<uint> ydist(area_yrange.lb(),
                                            area_yrange.ub() - 1);
  uint count = 0;

  do {
    rmath::vector2u loc = (*m_grid.origin()).loc();
    rel_x = area_xrange.span() > 0 ? xdist(m_rng) : m_grid.index_bases()[0];
    rel_y = area_xrange.span() > 0 ? ydist(m_rng) : m_grid.index_bases()[1];
    abs_x = rel_x + loc.x();
    abs_y = rel_y + loc.y();
  } while (std::any_of(entities.begin(), entities.end(), [&](const auto* ent) {
    return entity_contains_coord(ent, abs_x, abs_y) &&
           count++ <= kMAX_DIST_TRIES;
  }));
  if (count <= kMAX_DIST_TRIES) {
    coordv = std::vector<uint>({rel_x, rel_y, abs_x, abs_y});
    return true;
  }
  return false;
} /* find_avail_coord() */

bool random_distributor::entity_contains_coord(
    const representation::multicell_entity* const entity,
    double abs_x,
    double abs_y) {
  auto movable =
      dynamic_cast<const representation::movable_cell_entity*>(entity);
  rmath::vector2d coord = rmath::uvec2dvec(rmath::vector2u(abs_x, abs_y),
                                           m_resolution);
  if (nullptr != movable) {
    auto ent_xspan = entity->xspan(movable->real_loc());
    auto ent_yspan = entity->yspan(movable->real_loc());
    ER_TRACE("(movable entity) rcoord=%s, xspan=%s,yspan=%s",
             coord.to_str().c_str(),
             ent_xspan.to_str().c_str(),
             ent_yspan.to_str().c_str());
    return ent_xspan.contains(coord.x()) && ent_yspan.contains(coord.y());
  }

  auto immovable =
      dynamic_cast<const representation::immovable_cell_entity*>(entity);
  ER_ASSERT(nullptr != immovable,
            "Cell entity is neither movable nor immovable");
  auto ent_xspan = entity->xspan(immovable->real_loc());
  auto ent_yspan = entity->yspan(immovable->real_loc());

  ER_TRACE("(immovable entity) rcoord=%s, xspan=%s,yspan=%s",
           coord.to_str().c_str(),
           ent_xspan.to_str().c_str(),
           ent_yspan.to_str().c_str());
  return ent_xspan.contains(coord.x()) && ent_yspan.contains(coord.y());
} /* entity_contains_coord() */

NS_END(block_dist, support, fordyca);
