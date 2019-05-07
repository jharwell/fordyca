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
#include "fordyca/controller/existing_cache_selector.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/ds/dpo_store.hpp"
#include "fordyca/fsm/cache_acquisition_validator.hpp"
#include "fordyca/repr/base_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
using cselm = controller::cache_sel_matrix;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_existing_cache_fsm::acquire_existing_cache_fsm(
    const controller::cache_sel_matrix* matrix,
    bool is_pickup,
    controller::saa_subsystem* const saa,
    ds::dpo_store* const store)
    : ER_CLIENT_INIT("fordyca.fsm.acquire_existing_cache"),
      acquire_goal_fsm(
          saa,
          acquire_goal_fsm::hook_list{
              .acquisition_goal = std::bind(
                  &acquire_existing_cache_fsm::acquisition_goal_internal,
                  this),
              .goal_select =
                  std::bind(&acquire_existing_cache_fsm::existing_cache_select,
                            this),
              .candidates_exist =
                  std::bind(&acquire_existing_cache_fsm::candidates_exist, this),

              .goal_acquired_cb =
                  std::bind(&acquire_existing_cache_fsm::cache_acquired_cb,

                            this,
                            std::placeholders::_1),
              .explore_term_cb = std::bind(
                  &acquire_existing_cache_fsm::cache_exploration_term_cb,

                  this),
              .goal_valid_cb =
                  std::bind(&acquire_existing_cache_fsm::cache_acquisition_valid,
                            this,
                            std::placeholders::_1,
                            std::placeholders::_2)}),
      mc_is_pickup(is_pickup),
      mc_matrix(matrix),
      mc_store(store),
      m_rd(std::chrono::system_clock::now().time_since_epoch().count()) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<acquire_existing_cache_fsm::acquisition_loc_type>
acquire_existing_cache_fsm::calc_acquisition_location(void) {
  controller::existing_cache_selector selector(mc_is_pickup,
                                               mc_matrix,
                                               &mc_store->caches());

  if (auto best = selector(mc_store->caches(),
                           saa_subsystem()->sensing()->position(),
                           saa_subsystem()->sensing()->tick())) {
    ER_INFO("Selected existing cache%d@%s/%s, utility=%f for acquisition",
            best->ent()->id(),
            best->ent()->real_loc().to_str().c_str(),
            best->ent()->discrete_loc().to_str().c_str(),
            best->density().last_result());
    /*
     * Now that we have the location of the best cache, we need to pick a random
     * point inside it to vector to. This helps a LOT with maximimizing caches'
     * potential for traffic/congestion regulation, because it does not require
     * that all robots be able to make it to the center/near center of the cache
     * in order to utilize it (this is more realistic too).
     */
    auto xrange = best->ent()->xspan(best->ent()->real_loc());
    auto yrange = best->ent()->yspan(best->ent()->real_loc());
    std::uniform_real_distribution<double> xrnd(xrange.lb(), xrange.ub());
    std::uniform_real_distribution<double> yrnd(yrange.lb(), yrange.ub());

    rmath::vector2d loc = rmath::vector2d(xrnd(m_rd), yrnd(m_rd));
    ER_INFO("Selected point %s inside cache%d: xrange=%s, yrange=%s",
            loc.to_str().c_str(),
            best->ent()->id(),
            xrange.to_str().c_str(),
            yrange.to_str().c_str());
    return boost::make_optional(std::make_pair(best->ent()->id(), loc));

  } else {
    /*
     * If this happens, all the caches we know of are too close for us to vector
     * to, or otherwise unsuitable.
     */
    return boost::optional<acquisition_loc_type>();
  }
} /* calc_acquisition_location() */

bool acquire_existing_cache_fsm::cache_exploration_term_cb(void) const {
  return saa_subsystem()->sensing()->cache_detected();
} /* cache_exploration_term_cb() */

boost::optional<acquire_goal_fsm::candidate_type> acquire_existing_cache_fsm::
    existing_cache_select(void) {
  if (auto selection = calc_acquisition_location()) {
    return boost::make_optional(
        acquire_goal_fsm::candidate_type(selection.get().second,
                                         vector_fsm::kCACHE_ARRIVAL_TOL,
                                         selection.get().first));
  }
  return boost::optional<acquire_goal_fsm::candidate_type>();
} /* existing_cache_select() */

__rcsw_pure bool acquire_existing_cache_fsm::candidates_exist(void) const {
  return !mc_store->caches().empty();
} /* candidates() */

bool acquire_existing_cache_fsm::cache_acquired_cb(bool explore_result) const {
  if (explore_result) {
    return m_by_exploration_ok;
  } else {
    if (saa_subsystem()->sensing()->cache_detected()) {
      return true;
    }
    ER_WARN("Robot arrived at goal, but no cache was detected.");
    return false;
  }
} /* cache_acquired_cb() */

__rcsw_const acquisition_goal_type
acquire_existing_cache_fsm::acquisition_goal_internal(void) const {
  return acquisition_goal_type::ekEXISTING_CACHE;
} /* acquisition_goal() */

bool acquire_existing_cache_fsm::cache_acquisition_valid(
    const rmath::vector2d& loc,
    uint id) const {
  return cache_acquisition_validator(&mc_store->caches(), mc_matrix)(
      loc, id, saa_subsystem()->sensing()->tick());
} /* cache_acquisition_valid() */

NS_END(controller, fordyca);
