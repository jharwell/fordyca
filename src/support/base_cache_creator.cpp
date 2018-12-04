/**
 * @file base_cache_creator.cpp
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
#include "fordyca/support/base_cache_creator.hpp"

#include "fordyca/ds/cell2D.hpp"
#include "fordyca/events/cell_cache_extent.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/representation/arena_cache.hpp"
#include "fordyca/support/loop_utils/loop_utils.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);
using ds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_cache_creator::base_cache_creator(ds::arena_grid* const grid,
                                       double cache_dim)
    : ER_CLIENT_INIT("fordyca.support.base_cache_creator"),
      m_cache_dim(cache_dim),
      m_grid(grid) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::unique_ptr<representation::arena_cache> base_cache_creator::create_single_cache(
    ds::block_list blocks,
    const rmath::vector2d& center) {
  ER_ASSERT(center.x() > 0 && center.y() > 0,
            "Center@%s is not positive definite",
            center.to_str().c_str());
  /*
   * The cell that will be the location of the new cache may already contain a
   * block. If so, it should be added to the list of blocks for the cache.
   */
  rmath::vector2u d = rmath::dvec2uvec(center, grid()->resolution());
  ds::cell2D& cell = m_grid->access<arena_grid::kCell>(d);
  if (cell.state_has_block()) {
    ER_ASSERT(nullptr != cell.block(),
              "Cell@%s does not have block",
              cell.loc().to_str().c_str());

    /*
     * If the host cell for the cache already contains a block, we don't want to
     * unconditionally add said block to the list of blocks for the new cache,
     * because if the cache was created using some function of free blocks in
     * the arena, then that block could already be in the list, and we would be
     * double adding it, which will cause problems later (dynamic cache
     * creation). However, it may also NOT be in the list of blocks to use for
     * the new cache, in which case we need to add in (static cache creation).
     */
    if (blocks.end() == std::find(blocks.begin(), blocks.end(), cell.block())) {
      /*
       * We use insert() instead of push_back() here so that it there was a
       * leftover block on the cell where a cache used to be that is also where
       * this cache is being created, it becomes the "front" of the cache, and
       * will be the first block picked up by a robot from the new cache. This
       * helps to ensure fairness/better statistics for the simulations.
       */
      blocks.insert(blocks.begin(), cell.block());
    }
  }

  /*
   * The cells for all blocks that will comprise the cache should be set to
   * cache extent, and all blocks be deposited in a single cell.
   */
  for (auto& block : blocks) {
    events::cell_empty op(block->discrete_loc());
    m_grid->access<arena_grid::kCell>(op.x(), op.y()).accept(op);
  } /* for(block..) */

  for (auto& block : blocks) {
    events::free_block_drop op(block, d, m_grid->resolution());
    m_grid->access<arena_grid::kCell>(op.x(), op.y()).accept(op);
  } /* for(block..) */

  std::string s =
      std::accumulate(blocks.begin(),
                      blocks.end(),
                      std::string(),
                      [&](const std::string& a,
                          const std::shared_ptr<representation::base_block>& b) {
                        return a + "b" + std::to_string(b->id()) + ",";
                      });
  ds::block_vector block_vec(blocks.begin(), blocks.end());
  auto ret = rcppsw::make_unique<representation::arena_cache>(
      m_cache_dim, m_grid->resolution(), center, block_vec, -1);
  ER_INFO("Create cache%d@%s/%s, xspan=%s,yspan=%s with %zu blocks [%s]",
          ret->id(),
          ret->real_loc().to_str().c_str(),
          ret->discrete_loc().to_str().c_str(),
          ret->xspan(ret->real_loc()).to_str().c_str(),
          ret->yspan(ret->real_loc()).to_str().c_str(),
          ret->n_blocks(),
          s.c_str());
  return ret;
} /* create_single_cache() */

base_cache_creator::deconflict_res_t base_cache_creator::deconflict_loc_boundaries(
    double cache_dim,
    const rmath::vector2u& center) const {
  /*
   * We need to be sure the center of the new cache is not near the arena
   * boundaries, in order to avoid all sorts of weird corner cases.
   */
  double x_max = grid()->xrsize() - cache_dim * 2;
  double x_min = cache_dim * 2;
  double y_max = grid()->yrsize() - cache_dim * 2;
  double y_min = cache_dim * 2;

  bool conflict = false;
  rmath::vector2u new_center = center;
  if (center.x() > x_max || center.x() < x_min) {
    new_center.x(std::max(x_min, std::min(x_max, static_cast<double>(center.x()))));
    conflict = true;
    ER_TRACE("Adjust center=%s -> %s for arena boundaries X",
             center.to_str().c_str(),
             new_center.to_str().c_str());
  }

  if (center.y() > y_max || center.y() < y_min) {
    new_center.y(std::max(y_min, std::min(y_max, static_cast<double>(center.y()))));
    conflict = true;
    ER_TRACE("Adjust center=%s -> %s for arena boundaries Y",
             center.to_str().c_str(),
             new_center.to_str().c_str());
  }
  return deconflict_res_t{conflict, new_center};
} /* deconflict_loc_boundaries() */


base_cache_creator::deconflict_res_t base_cache_creator::
    deconflict_loc_entity(const representation::multicell_entity* ent,
                          const rmath::vector2d& ent_loc,
                          const rmath::vector2u& center) const {
  std::uniform_real_distribution<double> xrnd(-1.0, 1.0);
  std::uniform_real_distribution<double> yrnd(-1.0, 1.0);
  rmath::vector2d center_r = rmath::uvec2dvec(center, m_grid->resolution());

  auto exc_xspan = ent->xspan(ent_loc);
  auto exc_yspan = ent->yspan(ent_loc);
  auto newc_xspan = ent->xspan(center_r);
  auto newc_yspan = ent->yspan(center_r);


  auto status = loop_utils::placement_conflict(center_r,
                                               ent->dims(),
                                               ent);
  rmath::vector2u new_center = center;

  ER_TRACE("xspan=%s ent%d@%s [xspan=%s center=%s]",
           newc_xspan.to_str().c_str(),
           ent->id(),
           ent_loc.to_str().c_str(),
           exc_xspan.to_str().c_str(),
           center.to_str().c_str());
  ER_TRACE("yspan=%s ent%d@%s [yspan=%s center=%s]",
           newc_yspan.to_str().c_str(),
           ent->id(),
           ent_loc.to_str().c_str(),
           exc_yspan.to_str().c_str(),
           center.to_str().c_str());

  /*
   * We move the cache by units of the grid size when we discover a conflict in
   * X or Y, in order to preserve having the block location be on an even
   * multiple of the grid size, which makes handling creation much easier.
   */
  double x_delta = std::copysign(m_grid->resolution(), xrnd(m_rng));
  double y_delta = std::copysign(m_grid->resolution(), yrnd(m_rng));


  if (status.x_conflict) {
    ER_TRACE("xspan=%s overlap ent%d@%s [xspan=%s center=%s], x_delta=%f",
             newc_xspan.to_str().c_str(),
             ent->id(),
             ent_loc.to_str().c_str(),
             exc_xspan.to_str().c_str(),
             center.to_str().c_str(),
             x_delta);
    new_center.x(new_center.x() + x_delta);
  }
  if (status.y_conflict) {
    ER_TRACE("yspan=%s overlap ent%d@%s [yspan=%s center=%s], y_delta=%f",
             newc_yspan.to_str().c_str(),
             ent->id(),
             ent_loc.to_str().c_str(),
             exc_yspan.to_str().c_str(),
             center.to_str().c_str(),
             y_delta);
    new_center.y(new_center.y() + y_delta);
  }
  return deconflict_res_t{status.x_conflict && status.y_conflict, new_center};
} /* deconflict_ent() */

void base_cache_creator::update_host_cells(ds::cache_vector& caches) {
  /*
   * To reset all cells covered by a cache's extent, we simply send them a
   * CACHE_EXTENT event. EXCEPT for the cell that hosted the actual cache,
   * because it is currently in the HAS_CACHE state as part of the cache
   * creation process and setting it here will trigger an assert later.
   */
  for (auto& cache : caches) {
    m_grid->access<arena_grid::kCell>(cache->discrete_loc()).entity(cache);

    auto xspan = cache->xspan(cache->real_loc());
    auto yspan = cache->yspan(cache->real_loc());
    uint xmin = static_cast<uint>(std::ceil(xspan.lb() / m_grid->resolution()));
    uint xmax = static_cast<uint>(std::ceil(xspan.ub() / m_grid->resolution()));
    uint ymin = static_cast<uint>(std::ceil(yspan.lb() / m_grid->resolution()));
    uint ymax = static_cast<uint>(std::ceil(yspan.ub() / m_grid->resolution()));

    for (uint i = xmin; i < xmax; ++i) {
      for (uint j = ymin; j < ymax; ++j) {
        rmath::vector2u c = rmath::vector2u(i, j);
        if (c != cache->discrete_loc()) {
          ER_ASSERT(cache->contains_point(
                        rmath::uvec2dvec(c, m_grid->resolution())),
                    "Cache%d does not contain point (%u, %u) within its extent",
                    cache->id(),
                    i,
                    j);
          auto& cell = m_grid->access<arena_grid::kCell>(i, j);
          ER_ASSERT(!cell.state_in_cache_extent(),
                    "Cell@(%u, %u) already in CACHE_EXTENT",
                    i,
                    j);
          events::cell_cache_extent e(c, cache);
          cell.accept(e);
        }
      } /* for(j..) */
    }   /* for(i..) */
  }     /* for(cache..) */
} /* update_host_cells() */

bool base_cache_creator::creation_sanity_checks(
    const ds::cache_vector& caches,
    const ds::block_list& free_blocks) const {
  bool ret = true;

  for (auto& c1 : caches) {
    auto c1_xspan = c1->xspan(c1->real_loc());
    auto c1_yspan = c1->yspan(c1->real_loc());

    for (auto& c2 : caches) {
      if (c1->id() == c2->id()) {
        continue;
      }
      for (auto& b : c1->blocks()) {
        if (c2->contains_block(b)) {
          ER_FATAL_SENTINEL("Block%d contained in both cache%d and cache%d",
                            b->id(),
                            c1->id(),
                            c2->id());
          ret = false;
        }
        auto c2_xspan = c2->xspan(c2->real_loc());
        auto c2_yspan = c2->yspan(c2->real_loc());
        if (c1_xspan.overlaps_with(c2_xspan) &&
            c1_yspan.overlaps_with(c2_yspan)) {
          ER_FATAL_SENTINEL(
              "Cache%d xspan=%s, yspan=%s overlaps cache%d "
              "xspan=%s,yspan=%s",
              c1->id(),
              c1_xspan.to_str().c_str(),
              c1_yspan.to_str().c_str(),
              c2->id(),
              c2_xspan.to_str().c_str(),
              c2_yspan.to_str().c_str());
          ret = false;
        }
      } /* for(&b..) */
    }   /* for(&c2..) */

    for (auto &b : free_blocks) {
      auto b_xspan = b->xspan(b->real_loc());
      auto b_yspan = b->yspan(b->real_loc());
      if (c1_xspan.overlaps_with(b_xspan) &&
          c1_yspan.overlaps_with(b_yspan)) {
        ER_FATAL_SENTINEL(
            "Cache%d xspan=%s, yspan=%s overlaps block%d "
            "xspan=%s,yspan=%s",
            c1->id(),
            c1_xspan.to_str().c_str(),
            c1_yspan.to_str().c_str(),
            b->id(),
            b_xspan.to_str().c_str(),
            b_yspan.to_str().c_str());
        ret = false;
      }
    } /* for(&b..) */
  }     /* for(&c1..) */

  return ret;
} /* creation_sanity_checks() */

NS_END(support, fordyca);
