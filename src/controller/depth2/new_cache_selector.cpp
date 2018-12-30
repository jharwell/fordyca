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
ds::const_dp_block_set::value_type new_cache_selector::operator()(
    const ds::dp_block_set& new_caches,
    const ds::dp_cache_set& existing_caches,
    const rmath::vector2d& position) const {
  ds::const_dp_block_set::value_type best(nullptr, {});
  ER_ASSERT(!new_caches.empty(), "No known new caches");

  double max_utility = 0.0;
  for (auto& c : new_caches) {
    if (new_cache_is_excluded(existing_caches, new_caches, c.ent())) {
      continue;
    }
    math::new_cache_utility u(c.ent()->real_loc(),
                              boost::get<rmath::vector2d>(
                                  mc_matrix->find(cselm::kNestLoc)->second));

    double utility = u.calc(position, c.density().last_result());
    ER_ASSERT(utility > 0.0, "Bad utility calculation");
    ER_DEBUG("Utility for new cache%d@%s/%s, density=%f: %f",
             c.ent()->id(),
             best.ent()->real_loc().to_str().c_str(),
             best.ent()->discrete_loc().to_str().c_str(),
             c.density().last_result(),
             utility);

    if (utility > max_utility) {
      best = c;
      max_utility = utility;
    }
  } /* for(new_cache..) */

  if (nullptr != best.ent()) {
    ER_INFO("Best utility: new cache%d@%s/%s: %f",
            best.ent()->id(),
            best.ent()->real_loc().to_str().c_str(),
            best.ent()->discrete_loc().to_str().c_str(),
            max_utility);
  } else {
    ER_WARN("No best new cache found: all known new caches excluded!");
  }

  return best;
} /* operator() */

bool new_cache_selector::new_cache_is_excluded(
    const ds::dp_cache_set& existing_caches,
    const ds::dp_block_set& blocks,
    const representation::base_block* const new_cache) const {
  double cache_prox =
      boost::get<double>(mc_matrix->find(cselm::kCacheProxDist)->second);
  double cluster_prox =
      boost::get<double>(mc_matrix->find(cselm::kClusterProxDist)->second);

  for (auto& ec : existing_caches) {
    double dist = (ec.ent()->real_loc() - new_cache->real_loc()).length();
    if (dist <= cache_prox) {
      ER_DEBUG(
          "Ignoring new cache%d@%s/%s: Too close to cache%d@%s/%s (%f <= %f)",
          new_cache->id(),
          new_cache->real_loc().to_str().c_str(),
          new_cache->discrete_loc().to_str().c_str(),
          ec.ent()->id(),
          ec.ent()->real_loc().to_str().c_str(),
          ec.ent()->discrete_loc().to_str().c_str(),
          dist,
          cache_prox);
      return true;
    }
  } /* for(&ec..) */

  /*
   * Because robots have imperfect knowledge of the environment, AND that
   * environment is constantly changing (whether by the actions of other robots
   * or in and of itself), any block that they know about MIGHT be part of a
   * larger block cluster (i.e. part of a single source/dual source block
   * distribution).
   *
   * So, we approximate a block distribution as a single block, and only choose
   * new caches that are sufficiently far from any potential clusters.
   */
  for (auto& b : blocks) {
    if (b.ent() == new_cache) {
      continue;
    }
    double dist = (b.ent()->real_loc() - new_cache->real_loc()).length();

    if (dist <= cluster_prox) {
      ER_DEBUG(
          "Ignoring new cache%d@%s/%s: Too close to potential block "
          "cluster@%s/%s (%f <= %f)",
          new_cache->id(),
          new_cache->real_loc().to_str().c_str(),
          new_cache->discrete_loc().to_str().c_str(),
          b.ent()->real_loc().to_str().c_str(),
          b.ent()->discrete_loc().to_str().c_str(),
          dist,
          cluster_prox);
      return true;
    }
  } /* for(&b..) */

  return false;
} /* new_cache_is_excluded() */

NS_END(depth2, controller, fordyca);
