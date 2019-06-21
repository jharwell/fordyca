/**
 * @file cache_acq_point_selector.cpp
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
#include "fordyca/fsm/cache_acq_point_selector.hpp"
#include <algorithm>
#include <list>

#include "fordyca/repr/base_cache.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d cache_acq_point_selector::operator()(
    const rmath::vector2d& robot_loc,
    const repr::base_cache* const cache,
    std::default_random_engine& rd) {
  /*
   * Now that we have the closest quadrant center, construct [X,Y] ranges around
   * the center to pick a random point in. Each quadrant is cache_dim / 2.0, so
   * quadrant center needs to be computed using cache_dim / 4.0.
   */
  auto ul_center = rmath::vector2d(
      cache->real_loc().x() - cache->xsize() / 4.0 + m_arrival_tol,
      cache->real_loc().y() + cache->ysize() / 4.0 - m_arrival_tol);

  auto ur_center = rmath::vector2d(
      cache->real_loc().x() + cache->xsize() / 4.0 - m_arrival_tol,
      cache->real_loc().y() + cache->ysize() / 4.0 - m_arrival_tol);

  auto lr_center = rmath::vector2d(
      cache->real_loc().x() + cache->xsize() / 4.0 - m_arrival_tol,
      cache->real_loc().y() - cache->ysize() / 4.0 + m_arrival_tol);
  auto ll_center = rmath::vector2d(
      cache->real_loc().x() - cache->xsize() / 4.0 + m_arrival_tol,
      cache->real_loc().y() - cache->ysize() / 4.0 + m_arrival_tol);

  /* find closest center */
  std::list<rmath::vector2d> centers = {
      ul_center, ur_center, lr_center, ll_center};
  auto closest = *std::min_element(
      centers.begin(), centers.end(), [&](const auto& c1, const auto& c2) {
        return (c1 - robot_loc).length() < (c2 - robot_loc).length();
      });

  auto xrange = rmath::ranged(closest.x() - cache->xsize() / 4.0,
                              closest.x() + cache->xsize() / 4.0);
  auto yrange = rmath::ranged(closest.y() - cache->ysize() / 4.0,
                              closest.y() + cache->ysize() / 4.0);

  std::uniform_real_distribution<double> xrnd(xrange.lb(), xrange.ub());
  std::uniform_real_distribution<double> yrnd(yrange.lb(), yrange.ub());
  rmath::vector2d loc(xrnd(rd), yrnd(rd));

  ER_ASSERT(cache->contains_point(loc),
            "Cache%d@%s/%s does not contain %s",
            cache->id(),
            cache->real_loc().to_str().c_str(),
            cache->discrete_loc().to_str().c_str(),
            loc.to_str().c_str());
  ER_INFO("Point=%s in cache%d: robot_loc=%s,xrange=%s,yrange=%s",
          loc.to_str().c_str(),
          cache->id(),
          robot_loc.to_str().c_str(),
          xrange.to_str().c_str(),
          yrange.to_str().c_str());
  return loc;
} /* operator() */

NS_END(fsm, fordyca);
