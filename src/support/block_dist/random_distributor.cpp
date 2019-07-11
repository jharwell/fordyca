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
#include "fordyca/repr/base_block.hpp"
#include "fordyca/repr/base_cache.hpp"
#include "fordyca/repr/unicell_immovable_entity.hpp"
#include "fordyca/support/loop_utils/loop_utils.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, block_dist);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
random_distributor::random_distributor(const ds::arena_grid::view& grid,
                                       rtypes::discretize_ratio resolution)
    : ER_CLIENT_INIT("fordyca.support.block_dist.random"),
      mc_resolution(resolution),
      mc_origin(grid.origin()->loc()),
      mc_xspan(mc_origin.x(), mc_origin.x() + grid.shape()[0]),
      mc_yspan(mc_origin.y(), mc_origin.y() + grid.shape()[1]),
      m_grid(grid) {
  ER_INFO("Area: xrange=%s,yrange=%s,resolution=%f",
          mc_xspan.to_str().c_str(),
          mc_yspan.to_str().c_str(),
          mc_resolution.v());
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool random_distributor::distribute_blocks(ds::block_vector& blocks,
                                           ds::const_entity_list& entities) {
  ER_INFO("Distributing %zu blocks in area: xrange=%s, yrange=%s",
          blocks.size(),
          mc_xspan.to_str().c_str(),
          mc_yspan.to_str().c_str());

  return std::all_of(blocks.begin(), blocks.end(), [&](auto& b) {
    return this->distribute_block(b, entities);
  });
} /* distribute_blocks() */

bool random_distributor::distribute_block(std::shared_ptr<repr::base_block>& block,
                                          ds::const_entity_list& entities) {
  ds::cell2D* cell = nullptr;
  auto coords = avail_coord_search(entities, block->dims());
  if (coords) {
    ER_INFO("Found coordinates for distribution: rel=%s, abs=%s",
            coords->rel.to_str().c_str(),
            coords->abs.to_str().c_str());

    cell = &m_grid[coords->rel.x()][coords->rel.y()];

    /*
     * You can only distribute blocks to cells that do not currently have
     * anything in them. If there is already something there, then our
     * distribution algorithm has a bug.
     */
    ER_ASSERT(!cell->state_has_block(),
              "Destination cell@%s already contains block%d",
              coords->abs.to_str().c_str(),
              cell->block()->id());
    ER_ASSERT(!cell->state_has_cache(),
              "Destination cell@%s already contains cache%d",
              coords->abs.to_str().c_str(),
              cell->cache()->id());
    ER_ASSERT(!cell->state_in_cache_extent(),
              "Destination cell part of cache extent");

    events::free_block_drop_visitor op(block, coords->abs, mc_resolution);
    op.visit(*cell);
    if (verify_block_dist(block.get(), entities, cell)) {
      ER_DEBUG("Block%d,ptr=%p distributed@%s/%s",
               block->id(),
               block.get(),
               block->rloc().to_str().c_str(),
               block->dloc().to_str().c_str());
      /*
       * Now that the block has been distributed, it is another entity that
       * needs to be avoided during subsequent distributions.
       */
      entities.push_back(block.get());
      return true;
    }
    ER_WARN("Failed to distribute block%d after finding distribution coord",
            block->id());
    return false;
  } else {
    ER_WARN("Unable to find distribution coordinates for block%d", block->id());
    return false;
  }
} /* distribute_block() */

__rcsw_pure bool random_distributor::verify_block_dist(
    const repr::base_block* const block,
    const ds::const_entity_list& entities,
    __rcsw_unused const ds::cell2D* const cell) {
  /* blocks should not be out of sight after distribution... */
  ER_CHECK(repr::base_block::kOutOfSightDLoc != block->dloc(),
           "Block%d discrete coord still out of sight after distribution",
           block->id());
  ER_CHECK(repr::base_block::kOutOfSightRLoc != block->rloc(),
           "Block%d real coord still out of sight after distribution",
           block->id());

  /* The cell it was distributed to should refer to it */
  ER_CHECK(block == cell->block().get(),
           "Block%d@%s not referenced by containing cell@%s",
           block->id(),
           block->rloc().to_str().c_str(),
           cell->loc().to_str().c_str());

  /* no entity should overlap with the block after distribution */
  for (auto& e : entities) {
    if (e == block) {
      continue;
    }
    auto status =
        loop_utils::placement_conflict(block->rloc(), block->dims(), e);
    ER_ASSERT(!(status.x_conflict && status.y_conflict),
              "Entity contains block%d@%s/%s after distribution",
              block->id(),
              block->rloc().to_str().c_str(),
              block->rloc().to_str().c_str());
  } /* for(&e..) */
  return true;

error:
  return false;
} /* verify_block_dist() */

boost::optional<random_distributor::coord_search_res_t> random_distributor::
    avail_coord_search(const ds::const_entity_list& entities,
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
    uint x = area_xrange.span() > 0 ? xdist(rng()) : m_grid.index_bases()[0];
    uint y = area_xrange.span() > 0 ? ydist(rng()) : m_grid.index_bases()[1];
    rel = {x, y};
    abs = {rel.x() + mc_origin.x(), rel.y() + mc_origin.y()};
  } while (std::any_of(entities.begin(), entities.end(), [&](const auto* ent) {
    rmath::vector2d abs_r = rmath::uvec2dvec(abs, mc_resolution.v());
    auto status = loop_utils::placement_conflict(abs_r, block_dim, ent);
    return status.x_conflict && status.y_conflict && count++ <= kMAX_DIST_TRIES;
  }));
  if (count <= kMAX_DIST_TRIES) {
    return boost::make_optional(coord_search_res_t{rel, abs});
  }
  return boost::optional<coord_search_res_t>();
} /* avail_coord_search() */

NS_END(block_dist, support, fordyca);
