/**
 * \file cache_center_calculator.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#include "cosm/ds/cell2D.hpp"
#include "cosm/events/cell2D_empty.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/foraging/repr/block_cluster.hpp"

#include "fordyca/support/utils/loop_utils.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_center_calculator::cache_center_calculator(cds::arena_grid* const grid,
                                                 rtypes::spatial_dist cache_dim)
    : ER_CLIENT_INIT("fordyca.support.depth2.cache_center_calculator"),
      mc_cache_dim(cache_dim),
      m_grid(grid) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<rmath::vector2u> cache_center_calculator::operator()(
    const cds::block2D_vectorno& c_cache_i_blocks,
    const cads::acache_vectorno& c_existing_caches,
    const cfds::block_cluster_vector& c_clusters,
    rmath::rng* rng) const {
  double sumx = std::accumulate(c_cache_i_blocks.begin(),
                                c_cache_i_blocks.end(),
                                0.0,
                                [](double sum, const auto& b) {
                                  return sum + b->rloc().x();
                                });
  double sumy = std::accumulate(c_cache_i_blocks.begin(),
                                c_cache_i_blocks.end(),
                                0.0,
                                [](double sum, const auto& b) {
                                  return sum + b->rloc().y();
                                });

  /* center is discretized real coordinates WITHOUT converting via resolution */
  rmath::vector2u center(static_cast<uint>(sumx / c_cache_i_blocks.size()),
                         static_cast<uint>(sumy / c_cache_i_blocks.size()));
  ER_DEBUG("Guess center=%s", center.to_str().c_str());

  /*
   * This needs to be done even if there are no other known caches or blocks,
   * because the guessed center might still be too close to the arena
   * boundaries.
   */
  if (auto new_center = deconflict_loc_boundaries(center)) {
    center = new_center.get();
  }

  ER_DEBUG("Deconflict caches=[%s]",
           rcppsw::to_string(c_existing_caches).c_str());

  /*
   * Every time we find an overlap we have to re-test all of the caches we've
   * already verified won't overlap with our new cache, because the move we just
   * made in x or y might have just caused an overlap.
   */
  uint i = 0;
  while (i++ < kOVERLAP_SEARCH_MAX_TRIES) {
    if (auto new_center =
            deconflict_loc(c_existing_caches, c_clusters, center, rng)) {
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
    ER_WARN("No conflict-free center found in %u tries: caches=[%s]",
            kOVERLAP_SEARCH_MAX_TRIES,
            rcppsw::to_string(c_existing_caches).c_str());
    return boost::optional<rmath::vector2u>();
  }

  return boost::make_optional(rmath::vector2u(center.x(), center.y()));
} /* calc_center() */

boost::optional<rmath::vector2u> cache_center_calculator::deconflict_loc(
    const cads::acache_vectorno& c_existing_caches,
    const cfds::block_cluster_vector& c_clusters,
    const rmath::vector2u& c_center,
    rmath::rng* rng) const {
  bool conflict = false;
  rmath::vector2u new_center = c_center;
  for (size_t i = 0; i < c_clusters.size(); ++i) {
    for (size_t j = 0; j < c_existing_caches.size(); ++j) {
      /* check arena boundaries */
      if (auto new_loc = deconflict_loc_boundaries(c_center)) {
        new_center = new_loc.get();
        conflict = true;
      }

      /* check the current cache */
      if (auto new_loc =
              deconflict_loc_entity(c_existing_caches[j], c_center, rng)) {
        new_center = new_loc.get();
        conflict = true;
      }
      /* check the current block cluster */
      if (auto new_loc = deconflict_loc_entity(c_clusters[i], c_center, rng)) {
        new_center = new_loc.get();
        conflict = true;
      }
    } /* for(j..) */
  }   /* for(i..) */

  return (conflict) ? boost::make_optional(new_center)
                    : boost::optional<rmath::vector2u>();
} /* deconflict_loc() */

boost::optional<rmath::vector2u> cache_center_calculator::deconflict_loc_boundaries(
    const rmath::vector2u& c_center) const {
  /*
   * We need to be sure the center of the new cache is not near the arena
   * boundaries, in order to avoid all sorts of weird corner cases.
   */
  double x_max = m_grid->xrsize() - mc_cache_dim.v();
  double x_min = mc_cache_dim.v();
  double y_max = m_grid->yrsize() - mc_cache_dim.v();
  double y_min = mc_cache_dim.v();

  rmath::rangeu xbounds(static_cast<uint>(std::ceil(x_min)),
                        static_cast<uint>(std::floor(x_max)));
  rmath::rangeu ybounds(static_cast<uint>(std::ceil(y_min)),
                        static_cast<uint>(std::floor(y_max)));
  bool conflict = false;
  rmath::vector2u new_center = c_center;

  if (!xbounds.contains(new_center.x())) {
    new_center.x(xbounds.wrap_value(new_center.x()));
    conflict = true;
    ER_TRACE("Move center=%s -> %s to be within X=[%f,%f]",
             c_center.to_str().c_str(),
             new_center.to_str().c_str(),
             x_min,
             x_max);
  }

  if (!ybounds.contains(new_center.y())) {
    new_center.y(ybounds.wrap_value(new_center.y()));
    conflict = true;
    ER_TRACE("Move center=%s -> %s to be within Y=[%f,%f]",
             c_center.to_str().c_str(),
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
    const crepr::entity2D* ent,
    const rmath::vector2u& center,
    rmath::rng* rng) const {
  /*
   * The cache center is already a "real" coordinate, just one that has been
   * truncated to an integer.
   */
  rmath::vector2d center_r(center.x(), center.y());

  auto exc_xspan = ent->xspan();
  auto exc_yspan = ent->yspan();
  auto newc_xspan = crepr::entity2D::xspan(center_r, mc_cache_dim.v());
  auto newc_yspan = crepr::entity2D::yspan(center_r, mc_cache_dim.v());

  ER_TRACE("cache: xspan=%s,center=%s/%s, ent%d: xspan=%s",
           newc_xspan.to_str().c_str(),
           center_r.to_str().c_str(),
           center.to_str().c_str(),
           ent->id().v(),
           exc_xspan.to_str().c_str());

  ER_TRACE("cache: yspan=%s,center=%s/%s, ent%d: yspan=%s",
           newc_yspan.to_str().c_str(),
           center_r.to_str().c_str(),
           center.to_str().c_str(),
           ent->id().v(),
           exc_yspan.to_str().c_str());

  rmath::vector2u new_center = center;

  /*
   * We move the cache by units of the grid size when we discover a conflict in
   * X or Y, in order to preserve having the block location be on an even
   * multiple of the grid size, which makes handling creation much easier.
   */
  double x_delta =
      std::copysign(m_grid->resolution().v(), rng->uniform(-1.0, 1.0));
  double y_delta =
      std::copysign(m_grid->resolution().v(), rng->uniform(-1.0, 1.0));

  /*
   * Need to pass cache dimensions rather than dimensions of the entity, which
   * may be a block.
   */
  auto status = utils::placement_conflict(
      center_r, rmath::vector2d(mc_cache_dim.v(), mc_cache_dim.v()), ent);

  if (status.x_conflict) {
    ER_TRACE("cache: xspan=%s,center=%s overlap ent%d: xspan=%s, x_delta=%f",
             newc_xspan.to_str().c_str(),
             center.to_str().c_str(),
             ent->id().v(),
             exc_xspan.to_str().c_str(),
             x_delta);
    new_center.x(static_cast<uint>(new_center.x() + x_delta));
  }
  if (status.y_conflict) {
    ER_TRACE("cache: yspan=%s,center=%s overlap ent%d: yspan=%s, y_delta=%f",
             newc_yspan.to_str().c_str(),
             center.to_str().c_str(),
             ent->id().v(),
             exc_yspan.to_str().c_str(),
             y_delta);
    new_center.y(static_cast<uint>(new_center.y() + y_delta));
  }
  return (status.x_conflict && status.y_conflict)
             ? boost::make_optional(new_center)
             : boost::optional<rmath::vector2u>();
} /* deconflict_loc_entity() */

NS_END(depth2, support, fordyca);
