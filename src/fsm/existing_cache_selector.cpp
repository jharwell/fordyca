/**
 * \file existing_cache_selector.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/fsm/existing_cache_selector.hpp"

#include "cosm/arena/repr/base_cache.hpp"

#include "fordyca/controller/cognitive/cache_sel_matrix.hpp"
#include "fordyca/fsm/cache_acq_validator.hpp"
#include "fordyca/math/existing_cache_utility.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
using cselm = controller::cognitive::cache_sel_matrix;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
existing_cache_selector::existing_cache_selector(
    bool is_pickup,
    const controller::cognitive::cache_sel_matrix* const matrix,
    const ds::dp_cache_map* cache_map)
    : ER_CLIENT_INIT("fordyca.fsm.existing_cache_selector"),
      mc_is_pickup(is_pickup),
      mc_matrix(matrix),
      mc_cache_map(cache_map) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
const carepr::base_cache*
existing_cache_selector::operator()(const ds::dp_cache_map& existing_caches,
                                    const rmath::vector2d& position,
                                    const rtypes::timestep& t) {
  const carepr::base_cache* best = nullptr;
  ER_ASSERT(!existing_caches.empty(), "No known existing caches");

  double max_utility = 0.0;
  for (const auto& c : existing_caches.const_values_range()) {
    fsm::cache_acq_validator validator(mc_cache_map, mc_matrix, mc_is_pickup);

    if (!validator(c.ent()->rcenter2D(), c.ent()->id(), t) ||
        cache_is_excluded(position, c.ent())) {
      continue;
    }
    math::existing_cache_utility u(
        c.ent()->rcenter2D(),
        boost::get<rmath::vector2d>(mc_matrix->find(cselm::kNestLoc)->second));

    double utility = u.calc(position, c.density(), c.ent()->n_blocks());
    ER_ASSERT(utility > 0.0, "Bad utility calculation");
    ER_DEBUG("Utility for existing_cache%d@%s/%s, density=%f: %f",
             c.ent()->id().v(),
             rcppsw::to_string(c.ent()->rcenter2D()).c_str(),
             rcppsw::to_string(c.ent()->dcenter2D()).c_str(),
             c.density().v(),
             utility);

    if (utility > max_utility) {
      best = c.ent();
      max_utility = utility;
    }
  } /* for(existing_cache..) */

  ER_CHECKI(nullptr != best,
            "Best utility: existing_cache%d@%s/%s w/%zu blocks: %f",
            best->id().v(),
            rcppsw::to_string(best->rcenter2D()).c_str(),
            rcppsw::to_string(best->dcenter2D()).c_str(),
            best->n_blocks(),
            max_utility);
  ER_CHECKD(nullptr != best,
            "No best existing cache found: all known caches excluded!");
  return best;
} /* operator()() */

bool existing_cache_selector::cache_is_excluded(
    const rmath::vector2d& position,
    const carepr::base_cache* const cache) const {
  /**
   * If a robot is currently IN a cache, and wants to pick up from/drop
   * into a cache, it should generally ignored the cache it is currently in,
   * otherwise you have the potential for a robot to endlessly pick up from
   * a cache, drop in the same cache ad infinitum.
   *
   * This threshold prevents that behavior, forcing robots to at least LEAVE
   * the cache, even if they will then immediately return to it.
   */
  if (cache->contains_point2D(position)) {
    ER_DEBUG("Ignoring cache%d@%s/%s: robot@%s inside it",
             cache->id().v(),
             rcppsw::to_string(cache->rcenter2D()).c_str(),
             rcppsw::to_string(cache->dcenter2D()).c_str(),
             position.to_str().c_str());
    return true;
  }

  std::vector<rtypes::type_uuid> exceptions;
  if (mc_is_pickup) {
    exceptions = boost::get<std::vector<rtypes::type_uuid>>(
        mc_matrix->find(cselm::kPickupExceptions)->second);
  } else {
    exceptions = boost::get<std::vector<rtypes::type_uuid>>(
        mc_matrix->find(cselm::kDropExceptions)->second);
  }

  if (std::any_of(exceptions.begin(), exceptions.end(), [&](auto& id) {
        return id == cache->id();
      })) {
    ER_DEBUG("Ignoring cache%d@%s/%s: On exception list",
             cache->id().v(),
             rcppsw::to_string(cache->rcenter2D()).c_str(),
             rcppsw::to_string(cache->dcenter2D()).c_str());
    return true;
  }

  return false;
} /* cache_is_excluded() */

NS_END(fsm, fordyca);
