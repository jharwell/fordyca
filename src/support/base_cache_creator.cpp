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
    : ER_CLIENT_INIT("fordyca.support.depth1.base_cache_creator"),
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
  rcppsw::math::dcoord2 d = math::rcoord_to_dcoord(center, grid()->resolution());
  ds::cell2D& cell = m_grid->access<arena_grid::kCell>(d);
  if (cell.state_has_block()) {
    ER_ASSERT(cell.block(), "Cell does not have block");

    /*
     * We use insert() instead of push_back() here so that it there was a
     * leftover block on the cell where a cache used to be that is also where
     * this cache is being created, it becomes the "front" of the cache, and
     * will be the first block picked up by a robot from the new cache. This
     * helps to ensure fairness/better statistics for the simulations.
     */
    blocks.insert(blocks.begin(), cell.block());
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
  ER_INFO("Create cache%d@%s [%u,%u], xspan=%s,yspan=%s with %zu blocks [%s]",
          ret->id(),
          ret->real_loc().to_str().c_str(),
          ret->discrete_loc().first,
          ret->discrete_loc().second,
          ret->xspan(ret->real_loc()).to_str().c_str(),
          ret->yspan(ret->real_loc()).to_str().c_str(),
          ret->n_blocks(),
          s.c_str());
  return ret;
} /* create_single_cache() */

base_cache_creator::deconflict_result_type base_cache_creator::deconflict_existing_cache(
    const representation::base_cache& cache,
    const rmath::vector2i& center) const {
  std::uniform_real_distribution<double> xrnd(-1.0, 1.0);
  std::uniform_real_distribution<double> yrnd(-1.0, 1.0);

  auto exc_xspan = cache.xspan(cache.real_loc());
  auto exc_yspan = cache.yspan(cache.real_loc());
  auto newc_xspan = cache.xspan(rmath::ivec2dvec(center));
  auto newc_yspan = cache.yspan(rmath::ivec2dvec(center));

  rmath::vector2i new_center = center;

  /*
   * We need to be sure the center of the new cache is not near the arena
   * boundaries, in order to avoid all sorts of weird corner cases.
   */
  double x_max = grid()->xrsize() - exc_xspan.span() * 2;
  double x_min = exc_xspan.span() * 2;
  double y_max = grid()->yrsize() - exc_yspan.span() * 2;
  double y_min = exc_yspan.span() * 2;

  /*
   * Now that our center is comfortably in the middle of the arena, we need to
   * deconflict it with all known caches. We move the cache by unings of the
   * cache size (all caches are the same size), in order to preserve having the
   * cache location be on an even multiple of the grid size, making sure it
   * still stays in the arena boundaries.
   */
  double x_delta = std::copysign(newc_xspan.span(), xrnd(m_rng));
  double y_delta = std::copysign(newc_yspan.span(), yrnd(m_rng));

  bool x_conflict = false;
  bool y_conflict = false;
  if (newc_xspan.overlaps_with(exc_xspan)) {
    ER_TRACE("xspan=%s overlap cache%d@%s [xspan=%s center=%s], x_delta=%f",
             newc_xspan.to_str().c_str(),
             cache.id(),
             cache.real_loc().to_str().c_str(),
             exc_xspan.to_str().c_str(),
             center.to_str().c_str(),
             x_delta);
    new_center.x(std::max(x_min, std::min(x_max, new_center.x() + x_delta)));
    x_conflict = true;
  } else if (newc_yspan.overlaps_with(exc_yspan)) {
    ER_TRACE("yspan=%s overlap cache%d@%s [yspan=%s center=%s], y_delta=%f",
             newc_yspan.to_str().c_str(),
             cache.id(),
             cache.real_loc().to_str().c_str(),
             exc_yspan.to_str().c_str(),
             center.to_str().c_str(),
             y_delta);
    new_center.y(std::max(y_min, std::min(y_max, new_center.y() + y_delta)));
    y_conflict = true;
  }
  return std::make_pair(x_conflict && y_conflict, new_center);
} /* deconflict_existing_cache() */

rmath::vector2i base_cache_creator::deconflict_arena_boundaries(
    double cache_dim,
    const rmath::vector2i& center) const {

  /*
   * We need to be sure the center of the new cache is not near the arena
   * boundaries, in order to avoid all sorts of weird corner cases.
   */
  double x_max = grid()->xrsize() - cache_dim * 2;
  double x_min = cache_dim * 2;
  double y_max = grid()->yrsize() - cache_dim * 2;
  double y_min = cache_dim * 2;

  int x = std::max(x_min, std::min(x_max, static_cast<double>(center.x())));
  int y = std::max(y_min, std::min(y_max, static_cast<double>(center.y())));
  return rmath::vector2i(x, y);
} /* deconflict_arena_boundaries() */

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
        rcppsw::math::dcoord2 c = rcppsw::math::dcoord2(i, j);
        if (c != cache->discrete_loc()) {
          ER_ASSERT(cache->contains_point(
                        math::dcoord_to_rcoord(c, m_grid->resolution())),
                    "Cache%d does not contain point (%u, %u) within its extent",
                    cache->id(),
                    i,
                    j);
          auto& cell = m_grid->access<arena_grid::kCell>(i, j);
          ER_ASSERT(!cell.state_in_cache_extent(),
                    "cell(%u, %u) already in CACHE_EXTENT",
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
    const ds::cache_vector& caches) const {
  bool ret = true;
  for (auto& c1 : caches) {
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
        auto c1_xspan = c1->xspan(c1->real_loc());
        auto c2_xspan = c2->xspan(c2->real_loc());
        auto c1_yspan = c1->yspan(c1->real_loc());
        auto c2_yspan = c2->yspan(c2->real_loc());
        if (c1_xspan.overlaps_with(c2_xspan) &&
            c1_yspan.overlaps_with(c2_yspan)) {
          ER_FATAL_SENTINEL("Cache%d xspan=%s, yspan=%s overlaps cache%d "
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
  }     /* for(&c1..) */
  return ret;
} /* creation_sanity_checks() */

NS_END(support, fordyca);
