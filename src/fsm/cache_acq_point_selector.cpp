/**
 * \file cache_acq_point_selector.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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

  ER_ASSERT(cache->contains_point(loc),
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
