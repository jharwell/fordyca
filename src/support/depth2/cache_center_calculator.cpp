/**
 * @file cache_center_calculator.cpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/support/depth2/cache_center_calculator.hpp"
#include <chrono>

#include "fordyca/ds/cell2D.hpp"
#include "fordyca/events/cell_cache_extent.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/repr/arena_cache.hpp"
#include "fordyca/support/loop_utils/loop_utils.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);
using ds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_center_calculator::cache_center_calculator(ds::arena_grid* const grid,
                                                 double cache_dim)
    : ER_CLIENT_INIT("fordyca.support.depth2.cache_center_calculator"),
      m_cache_dim(cache_dim),
      m_grid(grid),
      m_rng(std::chrono::system_clock::now().time_since_epoch().count()) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<rmath::vector2u> cache_center_calculator::operator()(
    const ds::block_list& cache_i_blocks,
    const ds::block_list& nc_blocks,
    const ds::cache_vector& existing_caches) const {
  double sumx = std::accumulate(cache_i_blocks.begin(),
                                cache_i_blocks.end(),
                                0.0,
                                [](double sum, const auto& b) {
                                  return sum + b->real_loc().x();
                                });
  double sumy = std::accumulate(cache_i_blocks.begin(),
                                cache_i_blocks.end(),
                                0.0,
                                [](double sum, const auto& b) {
                                  return sum + b->real_loc().y();
                                });

  /* center is discretized real coordinates WITHOUT converting via resolution */
  rmath::vector2u center(static_cast<uint>(sumx / cache_i_blocks.size()),
                         static_cast<uint>(sumy / cache_i_blocks.size()));
  ER_DEBUG("Guess center=%s", center.to_str().c_str());

  /*
   * This needs to be done even if there are no other known caches or blocks,
   * because the guessed center might still be too close to the arena
   * boundaries.
   */
  if (auto new_center = deconflict_loc_boundaries(center)) {
    center = new_center.get();
  }

  ER_DEBUG("Deconflict caches=[%s]", rcppsw::to_string(existing_caches).c_str());
  ER_DEBUG("Deconflict blocks=[%s]", rcppsw::to_string(nc_blocks).c_str());

  /*
   * Every time we find an overlap we have to re-test all of the caches/blocks
   * we've already verified won't overlap with our new cache, because the move
   * we just made in x or y might have just caused an overlap.
   */
  uint i = 0;
  while (i++ < kOVERLAP_SEARCH_MAX_TRIES) {
    if (auto new_center = deconflict_loc(nc_blocks, existing_caches, center)) {
      center = new_center.get();
    } else {
      break;
    }
  } /* while(i..) */

  /*
   * We have a set # of tries to fiddle with the new cache center, and if we
   * can't find anything conflict free in that many, bail out.
   */
  if (i >= kOVERLAP_SEARCH_MAX_TRIES) {
    ER_WARN(
        "No conflict-free center found in %u tries: caches=[%s],blocks=[%s]",
        kOVERLAP_SEARCH_MAX_TRIES,
        rcppsw::to_string(existing_caches).c_str(),
        rcppsw::to_string(cache_i_blocks).c_str());
    return boost::optional<rmath::vector2u>();
  }

  return boost::make_optional(rmath::vector2u(center.x(), center.y()));
} /* calc_center() */

boost::optional<rmath::vector2u> cache_center_calculator::deconflict_loc(
    const ds::block_list& nc_blocks,
    const ds::cache_vector& existing_caches,
    const rmath::vector2u& center) const {
  bool conflict = false;
  rmath::vector2u new_center = center;
  for (size_t j = 0; j < existing_caches.size(); ++j) {
    if (auto new_loc = deconflict_loc_boundaries(center)) {
      new_center = new_loc.get();
      conflict = true;
    }

    if (auto new_loc = deconflict_loc_entity(
            existing_caches[j].get(), existing_caches[j]->real_loc(), center)) {
      new_center = new_loc.get();
      conflict = true;
    }
  } /* for(j..) */

  ER_DEBUG("Deconflict blocks=[%s]", rcppsw::to_string(nc_blocks).c_str());
  for (auto& b : nc_blocks) {
    if (auto new_loc = deconflict_loc_boundaries(center)) {
      new_center = new_loc.get();
      conflict = true;
    }
    if (auto new_loc = deconflict_loc_entity(b.get(), b->real_loc(), center)) {
      new_center = new_loc.get();
      conflict = true;
    }
  } /* for(j..) */

  return (conflict) ? boost::make_optional(new_center)
                    : boost::optional<rmath::vector2u>();
} /* deconflict_loc() */

boost::optional<rmath::vector2u> cache_center_calculator::deconflict_loc_boundaries(
    const rmath::vector2u& center) const {
  /*
   * We need to be sure the center of the new cache is not near the arena
   * boundaries, in order to avoid all sorts of weird corner cases.
   */
  double x_max = m_grid->xrsize() - m_cache_dim * 2;
  double x_min = m_cache_dim * 2;
  double y_max = m_grid->yrsize() - m_cache_dim * 2;
  double y_min = m_cache_dim * 2;

  rmath::rangeu xbounds(static_cast<uint>(std::ceil(x_min)),
                        static_cast<uint>(std::floor(x_max)));
  rmath::rangeu ybounds(static_cast<uint>(std::ceil(y_min)),
                        static_cast<uint>(std::floor(y_max)));
  bool conflict = false;
  rmath::vector2u new_center = center;

  if (!xbounds.contains(new_center.x())) {
    new_center.x(xbounds.wrap_value(new_center.x()));
    conflict = true;
    ER_TRACE("Move center=%s -> %s to be within X=[%f,%f]",
             center.to_str().c_str(),
             new_center.to_str().c_str(),
             x_min,
             x_max);
  }

  if (!ybounds.contains(new_center.y())) {
    new_center.y(ybounds.wrap_value(new_center.y()));
    conflict = true;
    ER_TRACE("Move center=%s -> %s to be within Y=[%f,%f]",
             center.to_str().c_str(),
             new_center.to_str().c_str(),
             y_min,
             y_max);
  }
  if (conflict) {
    ER_ASSERT(new_center.x() <= x_max && new_center.x() >= x_min &&
                  new_center.y() <= y_max && new_center.y() >= y_min,
              "New center=%s violates bounds constraints: X=[%f,%f],Y=[%f,%f]",
              new_center.to_str().c_str(),
              x_min,
              x_max,
              y_min,
              y_max);
    return boost::make_optional(new_center);
  }
  return boost::optional<rmath::vector2u>();
} /* deconflict_loc_boundaries() */

boost::optional<rmath::vector2u> cache_center_calculator::deconflict_loc_entity(
    const repr::multicell_entity* ent,
    const rmath::vector2d& ent_loc,
    const rmath::vector2u& center) const {
  /*
   * The cache center is already a "real" coordinate, just one that has been
   * truncated to an integer.
   */
  rmath::vector2d center_r(center.x(), center.y());

  auto exc_xspan = ent->xspan(ent_loc);
  auto exc_yspan = ent->yspan(ent_loc);
  auto newc_xspan = repr::multicell_entity::xspan(center_r, m_cache_dim);
  auto newc_yspan = repr::multicell_entity::yspan(center_r, m_cache_dim);

  ER_TRACE("cache: xspan=%s,center=%s/%s, ent%d@%s: xspan=%s",
           newc_xspan.to_str().c_str(),
           center_r.to_str().c_str(),
           center.to_str().c_str(),
           ent->id(),
           ent_loc.to_str().c_str(),
           exc_xspan.to_str().c_str());

  ER_TRACE("cache: yspan=%s,center=%s/%s, ent%d@%s: yspan=%s",
           newc_yspan.to_str().c_str(),
           center_r.to_str().c_str(),
           center.to_str().c_str(),
           ent->id(),
           ent_loc.to_str().c_str(),
           exc_yspan.to_str().c_str());

  rmath::vector2u new_center = center;

  /*
   * We move the cache by units of the grid size when we discover a conflict in
   * X or Y, in order to preserve having the block location be on an even
   * multiple of the grid size, which makes handling creation much easier.
   */
  std::uniform_real_distribution<double> xrnd(-1.0, 1.0);
  std::uniform_real_distribution<double> yrnd(-1.0, 1.0);
  double x_delta = std::copysign(m_grid->resolution(), xrnd(m_rng));
  double y_delta = std::copysign(m_grid->resolution(), yrnd(m_rng));

  /*
   * Need to pass cache dimensions rather than dimensions of the entity, which
   * may be a block.
   */
  auto status = loop_utils::placement_conflict(
      center_r, rmath::vector2d(m_cache_dim, m_cache_dim), ent);

  if (status.x_conflict) {
    ER_TRACE("cache: xspan=%s,center=%s overlap ent%d@%s: xspan=%s, x_delta=%f",
             newc_xspan.to_str().c_str(),
             center.to_str().c_str(),
             ent->id(),
             ent_loc.to_str().c_str(),
             exc_xspan.to_str().c_str(),
             x_delta);
    new_center.x(static_cast<uint>(new_center.x() + x_delta));
  }
  if (status.y_conflict) {
    ER_TRACE("cache: yspan=%s,center=%s overlap ent%d@%s: yspan=%s, y_delta=%f",
             newc_yspan.to_str().c_str(),
             center.to_str().c_str(),
             ent->id(),
             ent_loc.to_str().c_str(),
             exc_yspan.to_str().c_str(),
             y_delta);
    new_center.y(static_cast<uint>(new_center.y() + y_delta));
  }
  return (status.x_conflict && status.y_conflict)
             ? boost::make_optional(new_center)
             : boost::optional<rmath::vector2u>();
} /* deconflict_loc_entity() */

NS_END(depth2, support, fordyca);
