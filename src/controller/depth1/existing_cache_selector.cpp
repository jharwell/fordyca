/**
 * @file existing_cache_selector.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/controller/depth1/existing_cache_selector.hpp"
#include "fordyca/controller/cache_sel_matrix.hpp"
#include "fordyca/math/existing_cache_utility.hpp"
#include "fordyca/representation/base_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
existing_cache_selector::existing_cache_selector(
    const cache_sel_matrix* const matrix)
    : ER_CLIENT_INIT("fordyca.controller.depth0.existing_cache_selector"),
      mc_matrix(matrix) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
representation::perceived_cache existing_cache_selector::calc_best(
    const std::list<representation::perceived_cache>& existing_caches,
    const rmath::vector2d& position) {
  representation::perceived_cache best;
  ER_ASSERT(!existing_caches.empty(), "No known existing caches");

  double max_utility = 0.0;
  for (auto& c : existing_caches) {
    /**
     * If a robot is currently IN a cache, and wants to pick up from/drop
     * into a cache, it should generally ignored the cache it is currently in,
     * otherwise you have the potential for a robot to endlessly pick up from
     * a cache, drop in the same cache ad infinitum.
     *
     * This threshold prevents that behavior, forcing robots to at least LEAVE
     * the cache, even if they will then immediately return to it.
     */
    if ((position - c.ent->real_loc()).length() <=
        std::max(c.ent->xsize(), c.ent->ysize())) {
      ER_WARN("Ignoring cache%d in search: robot currently inside it",
              c.ent->id());
      continue;
    }
    math::existing_cache_utility u(c.ent->real_loc(),
                                   boost::get<rmath::vector2d>(
                                       mc_matrix->find("nest_loc")->second));

    double utility =
        u.calc(position, c.density.last_result(), c.ent->n_blocks());
    ER_ASSERT(utility > 0.0, "Bad utility calculation");
    ER_DEBUG("Utility for existing_cache%d loc=(%u, %u), density=%f: %f",
             c.ent->id(),
             c.ent->discrete_loc().first,
             c.ent->discrete_loc().second,
             c.density.last_result(),
             utility);

    if (utility > max_utility) {
      best = c;
      max_utility = utility;
    }
  } /* for(existing_cache..) */

  if (nullptr != best.ent) {
    ER_INFO("Best utility: existing_cache%d@%s [%u, %u]: %f",
            best.ent->id(),
            best.ent->real_loc().to_str().c_str(),
            best.ent->discrete_loc().first,
            best.ent->discrete_loc().second,
            max_utility);
  } else {
    ER_WARN("No best cache found: all known caches too close!");
  }

  return best;
} /* calc_best() */

NS_END(depth1, controller, fordyca);
