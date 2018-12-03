/**
 * @file block_to_existing_cache_fsm.cpp
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
#include "fordyca/fsm/acquire_existing_cache_fsm.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/controller/depth1/existing_cache_selector.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/ds/perceived_arena_map.hpp"
#include "fordyca/representation/base_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_existing_cache_fsm::acquire_existing_cache_fsm(
    const controller::cache_sel_matrix* matrix,
    bool is_pickup,
    controller::saa_subsystem* const saa,
    ds::perceived_arena_map* const map)
    : ER_CLIENT_INIT("fordyca.fsm.acquire_existing_cache"),
      acquire_goal_fsm(
          saa,
          std::bind(&acquire_existing_cache_fsm::acquisition_goal_internal,
                    this),
          std::bind(&acquire_existing_cache_fsm::candidates_exist, this),
          std::bind(&acquire_existing_cache_fsm::existing_cache_select, this),
          std::bind(&acquire_existing_cache_fsm::cache_acquired_cb,
                    this,
                    std::placeholders::_1),
          std::bind(&acquire_existing_cache_fsm::cache_exploration_term_cb,
                    this)),
      mc_is_pickup(is_pickup),
      mc_matrix(matrix),
  mc_map(map) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool acquire_existing_cache_fsm::calc_acquisition_location(
    rmath::vector2d* const loc) {
  controller::depth1::existing_cache_selector selector(mc_is_pickup, mc_matrix);
  representation::perceived_cache best =
      selector.calc_best(mc_map->perceived_caches(),
                         saa_subsystem()->sensing()->position());
  /*
   * If this happens, all the caches we know of are too close for us to vector
   * to, or otherwise unsuitable.
   */
  if (nullptr == best.ent) {
    return false;
  }
  ER_INFO("Selected existing cache%d@%s/%s, utility=%f for acquisition",
          best.ent->id(),
          best.ent->real_loc().to_str().c_str(),
          best.ent->discrete_loc().to_str().c_str(),
          best.density.last_result());

  /*
   * Now that we have the location of the best cache, we need to pick a random
   * point inside it to vector to. This helps a LOT with maximimizing caches'
   * potential for traffic/congestion regulation, because it does not require
   * that all robots be able to make it to the center/near center of the cache
   * in order to utilize it (this is more realistic too).
   */
  auto xrange = best.ent->xspan(best.ent->real_loc());
  auto yrange = best.ent->yspan(best.ent->real_loc());
  std::uniform_real_distribution<double> xrnd(xrange.lb(), xrange.ub());
  std::uniform_real_distribution<double> yrnd(yrange.lb(), yrange.ub());

  *loc = rmath::vector2d(xrnd(m_rd), yrnd(m_rd));
  ER_INFO("Selected point %s inside cache%d: xrange=%s, yrange=%s",
          loc->to_str().c_str(),
          best.ent->id(),
          xrange.to_str().c_str(),
          yrange.to_str().c_str());
  return true;
} /* calc_acquisition_location() */

bool acquire_existing_cache_fsm::cache_exploration_term_cb(void) const {
  return saa_subsystem()->sensing()->cache_detected();
} /* cache_exploration_term_cb() */

acquire_goal_fsm::candidate_type acquire_existing_cache_fsm::existing_cache_select(
    void) {
  rmath::vector2d loc;
  if (!calc_acquisition_location(&loc)) {
    return acquire_goal_fsm::candidate_type(false, rmath::vector2d(), -1);
  } else {
    return acquire_goal_fsm::candidate_type(true,
                                            loc,
                                            vector_fsm::kCACHE_ARRIVAL_TOL);
  }
} /* existing_cache_select() */

bool acquire_existing_cache_fsm::candidates_exist(void) const {
  return !mc_map->perceived_caches().empty();
} /* candidates() */

bool acquire_existing_cache_fsm::cache_acquired_cb(bool explore_result) const {
  if (explore_result) {
    ER_ASSERT(saa_subsystem()->sensing()->cache_detected(),
              "No cache detected after successful exploration?");
    return true;
  } else {
    if (saa_subsystem()->sensing()->cache_detected()) {
      return true;
    }
    ER_WARN("Robot arrived at goal, but no cache was detected.");
    return false;
  }
} /* cache_acquired_cb() */

acquisition_goal_type acquire_existing_cache_fsm::acquisition_goal_internal(
    void) const {
  return acquisition_goal_type::kExistingCache;
} /* acquisition_goal() */

NS_END(controller, fordyca);
