/**
 * \file cache_acq_point_selector.cpp
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
#include "fordyca/fsm/cache_acq_point_selector.hpp"

#include <algorithm>
#include <list>

#include "cosm/arena/repr/base_cache.hpp"

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
    RCPPSW_UNUSED const rmath::vector2d& robot_loc,
    const carepr::base_cache* const cache,
    rmath::rng* rng) {
  auto xspan = cache->xrspan();
  auto yspan = cache->yrspan();
  auto xrange =
      rmath::ranged(xspan.lb() + m_arrival_tol, xspan.ub() - m_arrival_tol);
  auto yrange =
      rmath::ranged(yspan.lb() + m_arrival_tol, yspan.ub() - m_arrival_tol);

  rmath::vector2d loc(rng->uniform(xrange), rng->uniform(yrange));

  ER_ASSERT(cache->contains_point2D(loc),
            "Cache%d@%s/%s with xspan=%s,yspan=%s does not contain %s",
            cache->id().v(),
            rcppsw::to_string(cache->rcenter2D()).c_str(),
            rcppsw::to_string(cache->dcenter2D()).c_str(),
            rcppsw::to_string(cache->xrspan()).c_str(),
            rcppsw::to_string(cache->yrspan()).c_str(),
            rcppsw::to_string(loc).c_str());
  ER_INFO("Point=%s in cache%d: robot_loc=%s,xrange=%s,yrange=%s",
          rcppsw::to_string(loc).c_str(),
          cache->id().v(),
          rcppsw::to_string(robot_loc).c_str(),
          rcppsw::to_string(xrange).c_str(),
          rcppsw::to_string(yrange).c_str());
  return loc;
} /* operator() */

NS_END(fsm, fordyca);
