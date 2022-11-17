/**
 * \file new_cache_selector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/cognitive/d2/new_cache_selector.hpp"

#include "cosm/arena/repr/base_cache.hpp"

#include "fordyca/controller/cognitive/cache_sel_matrix.hpp"
#include "fordyca/math/new_cache_utility.hpp"
#include "fordyca/subsystem/perception/ds/dp_block_map.hpp"
#include "fordyca/subsystem/perception/ds/dp_cache_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2);
using cselm = cache_sel_matrix;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
new_cache_selector::new_cache_selector(
    const controller::cognitive::cache_sel_matrix* const csel_matrix)
    : ER_CLIENT_INIT("fordyca.controller.d2.new_cache_selector"),
      mc_matrix(csel_matrix) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
const crepr::base_block3D*
new_cache_selector::operator()(const fspds::dp_block_map& new_caches,
                               const fspds::dp_cache_map& existing_caches,
                               const rmath::vector2d& position) const {
  const crepr::base_block3D* best = nullptr;
  ER_ASSERT(!new_caches.empty(), "No known new caches");

  double max_utility = 0.0;
  for (const auto& c : new_caches.values_range()) {
    if (new_cache_is_excluded(existing_caches, new_caches, c.ent())) {
      continue;
    }
    /*
     * Use the center rather than the anchor to get a utility unaffected by the
     * relative position of the block and the robot
     */
    math::new_cache_utility u(
        c.ent()->rcenter2D(),
        std::get<rmath::vector2d>(mc_matrix->find(cselm::kNestLoc)->second));

    double utility = u.calc(position, c.density());
    ER_ASSERT(utility > 0.0, "Bad utility calculation");
    ER_DEBUG("Utility for new cache%d@%s/%s, density=%f: %f",
             c.ent()->id().v(),
             rcppsw::to_string(c.ent()->ranchor2D()).c_str(),
             rcppsw::to_string(c.ent()->danchor2D()).c_str(),
             c.density().v(),
             utility);

    if (utility > max_utility) {
      best = c.ent();
      max_utility = utility;
    }
  } /* for(new_cache..) */

  ER_CONDI(nullptr != best,
           "Best utility: new cache%d@%s/%s: %f",
           best->id().v(),
           rcppsw::to_string(best->ranchor2D()).c_str(),
           rcppsw::to_string(best->danchor2D()).c_str(),
           max_utility);

  ER_CONDW(nullptr == best,
           "No best new cache found: all known new caches excluded!");
  return best;
} /* operator() */

bool new_cache_selector::new_cache_is_excluded(
    const fspds::dp_cache_map& existing_caches,
    const fspds::dp_block_map& blocks,
    const crepr::base_block3D* const new_cache) const {
  auto cache_prox = std::get<rspatial::euclidean_dist>(
      mc_matrix->find(cselm::kCacheProxDist)->second);
  auto cluster_prox = std::get<rspatial::euclidean_dist>(
      mc_matrix->find(cselm::kClusterProxDist)->second);

  /*
   * Use the center rather than the anchor to get a distance unaffected by the
   * relative position of an existing cache and new cache.
   */
  for (const auto& ec : existing_caches.values_range()) {
    double dist = (ec.ent()->rcenter2D() - new_cache->rcenter2D()).length();
    if (cache_prox >= dist) {
      ER_DEBUG("Ignoring new cache%d@%s/%s: Too close to cache%d@%s/%s (%f <= "
               "%f)",
               new_cache->id().v(),
               rcppsw::to_string(new_cache->ranchor2D()).c_str(),
               rcppsw::to_string(new_cache->danchor2D()).c_str(),
               ec.ent()->id().v(),
               rcppsw::to_string(ec.ent()->rcenter2D()).c_str(),
               rcppsw::to_string(ec.ent()->dcenter2D()).c_str(),
               dist,
               cache_prox.v());
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
  for (const auto& b : blocks.values_range()) {
    if (b.ent() == new_cache) {
      continue;
    }
    double dist = (b.ent()->rcenter2D() - new_cache->rcenter2D()).length();

    if (cluster_prox >= dist) {
      ER_DEBUG("Ignoring new cache%d@%s/%s: Too close to potential block "
               "cluster@%s/%s (%f <= %f)",
               new_cache->id().v(),
               rcppsw::to_string(new_cache->ranchor2D()).c_str(),
               rcppsw::to_string(new_cache->danchor2D()).c_str(),
               rcppsw::to_string(b.ent()->ranchor2D()).c_str(),
               rcppsw::to_string(b.ent()->danchor2D()).c_str(),
               dist,
               cluster_prox.v());
      return true;
    }
  } /* for(&b..) */

  return false;
} /* new_cache_is_excluded() */

NS_END(cognitive, d2, controller, fordyca);
