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
#include "fordyca/support/d2/cache_center_calculator.hpp"

#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/foraging/repr/block_cluster.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/repr/nest.hpp"
#include "cosm/spatial/conflict_checker.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, d2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_center_calculator::cache_center_calculator(
    cds::arena_grid* const grid,
    const rtypes::spatial_dist& cache_dim,
    const cads::nest_vectorro& c_nests,
    const cfds::block3D_cluster_vector& c_clusters)
    : ER_CLIENT_INIT("fordyca.support.d2.cache_center_calculator"),
      mc_cache_dim(cache_dim),
      mc_nests(c_nests),
      mc_clusters(c_clusters),
      m_grid(grid) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<rmath::vector2d> cache_center_calculator::operator()(
    const cds::block3D_vectorno& c_cache_i_blocks,
    const cads::acache_vectorno& c_existing_caches,
    rmath::rng* rng) const {
  ER_TRACE("Existing caches: [%s]", rcppsw::to_string(c_existing_caches).c_str());
  ER_TRACE("Cache_i blocks: [%s]", rcppsw::to_string(c_cache_i_blocks).c_str());
  ER_TRACE("Block clusters: [%s]", rcppsw::to_string(mc_clusters).c_str());
  auto sum = std::accumulate(c_cache_i_blocks.begin(),
                             c_cache_i_blocks.end(),
                             rmath::vector2d(),
                             [](rmath::vector2d accum, const auto& b) {
                               return accum + b->rcenter2D();
                             });

  /*
   * After real -> discrete -> real coordinates transformation to the averaged
   * location of the blocks, we have the location of the LL corner of the cell
   * that is our guess for the cache center. We need to move it up to the center
   * of that cell so that all the xspan/yspan calculations come out correct in
   * all cases.
   */
  rmath::vector2d offset(m_grid->resolution().v() / 2.0,
                         m_grid->resolution().v() / 2.0);

  auto ll =
      rmath::dvec2zvec(sum / c_cache_i_blocks.size(), m_grid->resolution().v());
  rmath::vector2d center =
      rmath::zvec2dvec(ll, m_grid->resolution().v()) + offset;

  ER_DEBUG("Guess center=%s", center.to_str().c_str());

  /*
   * Every time we find an overlap we have to re-test EVERYTHING we've already
   * verified won't overlap with our new cache, because the move we just made in
   * x or y might have just caused an overlap.
   */
  size_t i = 0;
  while (i++ < kOVERLAP_SEARCH_MAX_TRIES) {
    ER_TRACE("Begin search pass %zu", i);
    if (auto new_center = deconflict_loc(c_existing_caches, center, rng)) {
      center = new_center.get();
      ER_TRACE("Search pass %zu unsuccessful: center=%s",
               i,
               rcppsw::to_string(center).c_str());
    } else {
      ER_TRACE("Search pass %zu successful: center=%s",
               i,
               rcppsw::to_string(center).c_str());
      break;
    }
  } /* while(i..) */

  /*
   * We have a set # of tries to fiddle with the new cache center, and if we
   * can't find anything conflict free in that many, bail out.
   */
  if (i >= kOVERLAP_SEARCH_MAX_TRIES) {
    ER_WARN("No conflict-free center found in %zu tries: caches=[%s] blocks=[%s] "
            "clusters=[%s], nests=[%s]",
            kOVERLAP_SEARCH_MAX_TRIES,
            rcppsw::to_string(c_existing_caches).c_str(),
            rcppsw::to_string(c_cache_i_blocks).c_str(),
            rcppsw::to_string(mc_clusters).c_str(),
            rcppsw::to_string(mc_nests).c_str());
    return boost::optional<rmath::vector2d>();
  }

  /* we found a center! */
  return boost::make_optional(center);
} /* calc_center() */

boost::optional<rmath::vector2d> cache_center_calculator::deconflict_loc(
    const cads::acache_vectorno& c_existing_caches,
    const rmath::vector2d& c_center,
    rmath::rng* rng) const {
  bool conflict = false;
  rmath::vector2d new_center = c_center;

  /* check arena boundaries */
  ER_TRACE("Check arena boundaries");
  if (auto new_loc = deconflict_loc_boundaries(c_center)) {
    new_center = new_loc.get();
    conflict = true;
  }

  /* check nests */
  for (size_t i = 0; i < mc_nests.size(); ++i) {
    ER_TRACE("Check nest%d: xspan=%s,yspan=%s",
             mc_nests[i]->id().v(),
             rcppsw::to_string(mc_nests[i]->xrspan()).c_str(),
             rcppsw::to_string(mc_nests[i]->yrspan()).c_str())
    if (auto new_loc = deconflict_loc_entity(mc_nests[i], c_center, rng)) {
      new_center = new_loc.get();
      conflict = true;
    }
  } /* for(i..) */

  /* check block clusters */
  for (size_t i = 0; i < mc_clusters.size(); ++i) {
    ER_TRACE("Check cluster%zu: xspan=%s,yspan=%s",
             i,
             rcppsw::to_string(mc_clusters[i]->xrspan()).c_str(),
             rcppsw::to_string(mc_clusters[i]->yrspan()).c_str())
    if (auto new_loc = deconflict_loc_entity(mc_clusters[i], c_center, rng)) {
      new_center = new_loc.get();
      conflict = true;
    }
  } /* for(i..) */

  /* check existing caches */
  for (size_t j = 0; j < c_existing_caches.size(); ++j) {
    ER_TRACE("Check cache%zu: xspan=%s,yspan=%s",
             j,
             rcppsw::to_string(c_existing_caches[j]->xrspan()).c_str(),
             rcppsw::to_string(c_existing_caches[j]->yrspan()).c_str())
    if (auto new_loc =
            deconflict_loc_entity(c_existing_caches[j], c_center, rng)) {
      new_center = new_loc.get();
      conflict = true;
    }
  } /* for(j..) */

  return (conflict) ? boost::make_optional(new_center)
                    : boost::optional<rmath::vector2d>();
} /* deconflict_loc() */

boost::optional<rmath::vector2d>
cache_center_calculator::deconflict_loc_boundaries(
    const rmath::vector2d& c_center) const {
  /*
   * We need to be sure the center of the new cache is not near the arena
   * boundaries, in order to avoid all sorts of weird corner cases.
   */
  rmath::ranged xbounds(mc_cache_dim.v(), m_grid->xrsize() - mc_cache_dim.v());
  rmath::ranged ybounds(mc_cache_dim.v(), m_grid->yrsize() - mc_cache_dim.v());
  bool conflict = false;
  rmath::vector2d new_center = c_center;

  if (!xbounds.contains(new_center.x())) {
    new_center.x(xbounds.wrap_value(new_center.x()));
    conflict = true;
    ER_TRACE("Move center=%s -> %s to be within X=%s",
             rcppsw::to_string(c_center).c_str(),
             rcppsw::to_string(new_center).c_str(),
             rcppsw::to_string(xbounds).c_str());
  }

  if (!ybounds.contains(new_center.y())) {
    new_center.y(ybounds.wrap_value(new_center.y()));
    conflict = true;
    ER_TRACE("Move center=%s -> %s to be within Y=%s",
             rcppsw::to_string(c_center).c_str(),
             rcppsw::to_string(new_center).c_str(),
             rcppsw::to_string(ybounds).c_str());
  }
  if (conflict) {
    ER_ASSERT(new_center.x() <= xbounds.ub() && new_center.x() >= xbounds.lb() &&
                  new_center.y() <= ybounds.ub() &&
                  new_center.y() >= ybounds.lb(),
              "New center=%s violates arena bounds constraints: X=%s,Y=%s",
              rcppsw::to_string(new_center).c_str(),
              rcppsw::to_string(xbounds).c_str(),
              rcppsw::to_string(ybounds).c_str());
    return boost::make_optional(new_center);
  }
  return boost::optional<rmath::vector2d>();
} /* deconflict_loc_boundaries() */

boost::optional<rmath::vector2d>
cache_center_calculator::deconflict_loc_entity(const crepr::entity2D* ent,
                                               const rmath::vector2d& c_center,
                                               rmath::rng* rng) const {
  RCPPSW_UNUSED auto dcenter =
      rmath::dvec2zvec(c_center, m_grid->resolution().v());
  auto exc_xspan = ent->xrspan();
  auto exc_yspan = ent->yrspan();
  auto newc_xspan = crepr::entity2D::xrspan(c_center, mc_cache_dim);
  auto newc_yspan = crepr::entity2D::yrspan(c_center, mc_cache_dim);

  ER_TRACE("cache: xspan=%s,center=%s/%s ent%d: xspan=%s",
           rcppsw::to_string(newc_xspan).c_str(),
           rcppsw::to_string(c_center).c_str(),
           rcppsw::to_string(dcenter).c_str(),
           ent->id().v(),
           rcppsw::to_string(exc_xspan).c_str());

  ER_TRACE("cache: yspan=%s,center=%s/%s ent%d: yspan=%s",
           rcppsw::to_string(newc_yspan).c_str(),
           rcppsw::to_string(c_center).c_str(),
           rcppsw::to_string(dcenter).c_str(),
           ent->id().v(),
           rcppsw::to_string(exc_yspan).c_str());

  rmath::vector2d new_center = c_center;

  /*
   * We move the cache by units of the grid size when we discover a conflict in
   * X or Y, in order to preserve having the cache location be on an even
   * multiple of the grid size, which makes handling creation much easier.
   */
  double x_delta =
      std::copysign(m_grid->resolution().v(), rng->uniform(-1.0, 1.0));
  double y_delta =
      std::copysign(m_grid->resolution().v(), rng->uniform(-1.0, 1.0));

  /*
   * The placement_conflict2D() function expects the LL anchor of the entity,
   * NOT the center, so we have to compute that.
   */
  rmath::vector2d cache_dim(mc_cache_dim.v(), mc_cache_dim.v());
  auto status = cspatial::conflict_checker::placement2D(
      c_center - cache_dim / 2.0, cache_dim, ent);

  if (status.x) {
    ER_TRACE("X conflict: cache xspan=%s,center=%s overlap ent%d: xspan=%s, "
             "x_delta=%f",
             rcppsw::to_string(newc_xspan).c_str(),
             rcppsw::to_string(c_center).c_str(),
             ent->id().v(),
             rcppsw::to_string(exc_xspan).c_str(),
             x_delta);
    new_center.x(new_center.x() + x_delta);
  }
  if (status.y) {
    ER_TRACE("Y conflict: cache yspan=%s,center=%s overlap ent%d: yspan=%s, "
             "y_delta=%f",
             rcppsw::to_string(newc_yspan).c_str(),
             rcppsw::to_string(c_center).c_str(),
             ent->id().v(),
             rcppsw::to_string(exc_yspan).c_str(),
             y_delta);
    new_center.y(new_center.y() + y_delta);
  }
  return (status.x && status.y) ? boost::make_optional(new_center)
                                : boost::optional<rmath::vector2d>();
} /* deconflict_loc_entity() */

NS_END(d2, support, fordyca);
