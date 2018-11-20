/**
 * @file new_cache_selector.cpp
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
#include "fordyca/controller/depth2/new_cache_selector.hpp"
#include "fordyca/controller/cache_sel_matrix.hpp"
#include "fordyca/math/new_cache_utility.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "fordyca/representation/perceived_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth2);
using cselm = cache_sel_matrix;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
new_cache_selector::new_cache_selector(
    const controller::cache_sel_matrix* const csel_matrix)
    : ER_CLIENT_INIT("fordyca.controller.depth2.new_cache_selector"),
      mc_matrix(csel_matrix) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
representation::perceived_block new_cache_selector::operator()(
    const ds::perceived_block_list& new_caches,
    const ds::cache_list& existing_caches,
    const rmath::vector2d& position) const {
  representation::perceived_block best;
  ER_ASSERT(!new_caches.empty(), "No known new caches");

  double max_utility = 0.0;
  for (auto& c : new_caches) {
    if (new_cache_is_excluded(existing_caches, c.ent.get())) {
      continue;
    }
    math::new_cache_utility u(c.ent->real_loc(),
                              boost::get<rmath::vector2d>(
                                  mc_matrix->find(cselm::kNestLoc)->second));

    double utility = u.calc(position, c.density.last_result());
    ER_ASSERT(utility > 0.0, "Bad utility calculation");
    ER_DEBUG("Utility for new cache%d@%s/%s, density=%f: %f",
             c.ent->id(),
             best.ent->real_loc().to_str().c_str(),
             best.ent->discrete_loc().to_str().c_str(),
             c.density.last_result(),
             utility);

    if (utility > max_utility) {
      best = c;
      max_utility = utility;
    }
  } /* for(new_cache..) */

  if (nullptr != best.ent) {
    ER_INFO("Best utility: new cache%d@%s/%s: %f",
            best.ent->id(),
            best.ent->real_loc().to_str().c_str(),
            best.ent->discrete_loc().to_str().c_str(),
            max_utility);
  } else {
    ER_WARN("No best new cache found: all known new caches excluded!");
  }

  return best;
} /* operator() */

bool new_cache_selector::new_cache_is_excluded(
    const ds::cache_list& existing_caches,
    const representation::base_block* const new_cache) const {
  double threshold =
      boost::get<double>(mc_matrix->find(cselm::kCacheProxDist)->second);
  for (auto &ec : existing_caches) {
    double prox = (ec->real_loc() - new_cache->real_loc()).length();
    if (prox <= threshold) {
      ER_DEBUG("Ignoring new cache%d@%s/%s: Too close to cache%d@%s/%s (%f <= %f)",
               new_cache->id(),
               new_cache->real_loc().to_str().c_str(),
               new_cache->discrete_loc().to_str().c_str(),
               ec->id(),
               ec->real_loc().to_str().c_str(),
               ec->discrete_loc().to_str().c_str(),
               prox,
               threshold);
      return true;
    }
  } /* for(&ec..) */


  return false;
} /* new_cache_is_excluded() */

NS_END(depth2, controller, fordyca);
