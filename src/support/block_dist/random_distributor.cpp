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

#include "fordyca/ds/cell2D.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/representation/immovable_cell_entity.hpp"
#include "fordyca/representation/multicell_entity.hpp"
#include "fordyca/support/loop_utils/loop_utils.hpp"

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
  __rcsw_unused rmath::vector2u loc = (*m_grid.origin()).loc();
  ER_INFO("Distributing %zu blocks in area: xrange=[%u-%lu], yrange=[%u-%lu]",
          blocks.size(),
          loc.x(),
          loc.x() + m_grid.shape()[0],
          loc.y(),
          loc.y() + m_grid.shape()[1]);

  return std::all_of(blocks.begin(), blocks.end(), [&](auto& b) {
    return this->distribute_block(b, entities);
  });
} /* distribute_blocks() */

bool random_distributor::distribute_block(
    std::shared_ptr<representation::base_block>& block,
    ds::const_entity_list& entities) {
  ds::cell2D* cell = nullptr;
  coord_search_res_t res = avail_coord_search(entities, block->dims());
  if (!res.status) {
    ER_WARN("Unable to find distribution coordinates for block%d", block->id());
    return false;
  }
  ER_INFO("Found coordinates for distribution: rel=%s, abs=%s",
          res.rel.to_str().c_str(),
          res.abs.to_str().c_str());

  cell = &m_grid[res.rel.x()][res.rel.y()];

  /*
   * You can only distribute blocks to cells that do not currently have
   * anything in them. If there is already something there, then our
   * distribution algorithm has a bug.
   */
  ER_ASSERT(!cell->state_has_block(), "Destination cell already contains block");
  ER_ASSERT(!cell->state_has_cache(), "Destination cell already contains cache");
  ER_ASSERT(!cell->state_in_cache_extent(),
            "Destination cell part of cache extent");

  events::free_block_drop op(block, res.abs, m_resolution);
  cell->accept(op);
  if (verify_block_dist(block.get(), entities, cell)) {
    ER_DEBUG("Block%d,ptr=%p distributed@%s/%s",
             block->id(),
             block.get(),
             block->real_loc().to_str().c_str(),
             block->discrete_loc().to_str().c_str());
    /*
     * Now that the block has been distributed, it is another entity that
     * needs to be avoided during subsequent distributions.
     */
    entities.push_back(block.get());
    return true;
  } else {
    ER_WARN("Failed to distribute block%d after finding distribution coord",
            block->id());
    return false;
  }
} /* distribute_block() */

__rcsw_pure bool random_distributor::verify_block_dist(
    const representation::base_block* const block,
    const ds::const_entity_list& entities,
    __rcsw_unused const ds::cell2D* const cell) {
  /* blocks should not be out of sight after distribution... */
  ER_CHECK(representation::base_block::kOutOfSightDLoc != block->discrete_loc(),
           "Block%d discrete coordinates still out of sight after "
           "distribution",
           block->id());
  ER_CHECK(representation::base_block::kOutOfSightRLoc != block->real_loc(),
           "Block%d real coordinates still out of sight after distribution",
           block->id());

  /* The cell it was distributed to should refer to it */
  ER_CHECK(block == cell->block().get(),
           "Block%d@%s not referenced by containing cell@%s",
           block->id(),
           block->real_loc().to_str().c_str(),
           cell->loc().to_str().c_str());

  /* no entity should overlap with the block after distribution */
  for (auto& e : entities) {
    if (e == block) {
      continue;
    }
    auto status =
        loop_utils::placement_conflict(block->real_loc(), block->dims(), e);
    ER_ASSERT(!(status.x_conflict && status.y_conflict),
              "Entity contains block%d@%s/%s after distribution",
              block->id(),
              block->real_loc().to_str().c_str(),
              block->real_loc().to_str().c_str());
  } /* for(&e..) */
  return true;

error:
  return false;
} /* verify_block_dist() */

random_distributor::coord_search_res_t random_distributor::avail_coord_search(
    const ds::const_entity_list& entities,
    const rmath::vector2d& block_dim) {
  rmath::vector2u rel;
  rmath::vector2u abs;
  rcppsw::math::rangeu area_xrange(m_grid.index_bases()[0], m_grid.shape()[0]);
  rcppsw::math::rangeu area_yrange(m_grid.index_bases()[1], m_grid.shape()[1]);

  /* -1 because we are working with array indices */
  std::uniform_int_distribution<uint> xdist(area_xrange.lb(),
                                            area_xrange.ub() - 1);
  std::uniform_int_distribution<uint> ydist(area_yrange.lb(),
                                            area_yrange.ub() - 1);
  uint count = 0;

  /*
   * Try to find an available set of relative+absolute coordinates such that if
   * the entity is placed there it will not overlap any other entities in the
   * arena. You only have a finite number of tries, for obvious reasons.
   */
  do {
    rmath::vector2u loc = (*m_grid.origin()).loc();
    uint x = area_xrange.span() > 0 ? xdist(m_rng) : m_grid.index_bases()[0];
    uint y = area_xrange.span() > 0 ? ydist(m_rng) : m_grid.index_bases()[1];
    rel = {x, y};
    abs = {rel.x() + loc.x(), rel.y() + loc.y()};
  } while (std::any_of(entities.begin(), entities.end(), [&](const auto* ent) {
    rmath::vector2d abs_r = rmath::uvec2dvec(abs, m_resolution);
    auto status = loop_utils::placement_conflict(abs_r, block_dim, ent);
    return status.x_conflict && status.y_conflict && count++ <= kMAX_DIST_TRIES;
  }));
  if (count <= kMAX_DIST_TRIES) {
    return {true, rel, abs};
  }
  return {false, {}, {}};
} /* avail_coord_search() */

NS_END(block_dist, support, fordyca);
