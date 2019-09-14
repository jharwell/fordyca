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
    rmath::rng* rng) {
  auto xspan = cache->xspan();
  auto yspan = cache->yspan();
  auto xrange =
      rmath::ranged(xspan.lb() + m_arrival_tol, xspan.ub() - m_arrival_tol);
  auto yrange =
      rmath::ranged(yspan.lb() + m_arrival_tol, yspan.ub() - m_arrival_tol);

  rmath::vector2d loc(rng->uniform(xrange), rng->uniform(yrange));

  ER_ASSERT(cache->contains_point(loc),
            "Cache%d@%s/%s with xspan=%s,yspan=%s does not contain %s",
            cache->id(),
            cache->rloc().to_str().c_str(),
            cache->dloc().to_str().c_str(),
            cache->xspan().to_str().c_str(),
            cache->yspan().to_str().c_str(),
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
