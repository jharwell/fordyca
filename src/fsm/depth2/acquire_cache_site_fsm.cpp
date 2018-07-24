/**
 * @file acquire_cache_site_fsm.cpp
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
#include "fordyca/fsm/depth2/acquire_cache_site_fsm.hpp"

#include "fordyca/controller/depth1/sensing_subsystem.hpp"
#include "fordyca/controller/depth2/cache_site_selector.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth2);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_cache_site_fsm::acquire_cache_site_fsm(
    std::shared_ptr<rcppsw::er::server>& server,
    const controller::cache_selection_matrix* csel_matrix,
    controller::saa_subsystem* const saa,
    representation::perceived_arena_map* const map)
    : acquire_goal_fsm(server,
                       saa,
                       map,
                       std::bind(&acquire_cache_site_fsm::site_detected_cb,
                                 this)),
      mc_matrix(csel_matrix) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
__rcsw_const bool acquire_cache_site_fsm::site_acquired_cb(bool explore_result) const {
  /*
   * At some point this will have some sanity checks/warnings about the chosen
   * cache site, but for now, just signal that everything is fine.
   */
  ER_ASSERT(!explore_result, "FATAL: Found cache site by exploring?");
  return true;
} /* site_acquired_cb() */

bool acquire_cache_site_fsm::acquire_known_goal(void) {
  /*
   * @todo In the future, it might be good to have the optimizer adapter class
   * return a set of cache sites (3-4?) so that if the first one does not work
   * out, we can try another, instead of having to immediately resort to
   * exploring.
   */

  /* Start vectoring towards our chosen site */
  if (!vector_fsm().task_running() && !vector_fsm().task_finished()) {
    controller::depth2::cache_site_selector s(server_ref(), mc_matrix);
    tasks::vector_argument v(
        vector_fsm::kCACHE_SITE_ARRIVAL_TOL,
        s.calc_best(std::list<representation::perceived_cache>(),
                    saa_subsystem()->sensing()->position()));
    explore_fsm().task_reset();
    vector_fsm().task_reset();
    vector_fsm().task_start(&v);
  }

  /* we are vectoring */
  if (!vector_fsm().task_finished()) {
    vector_fsm().task_execute();
  }

  if (vector_fsm().task_finished()) {
    vector_fsm().task_reset();
    return site_acquired_cb(false);
  }
  return false;
} /* acquire_known_goal() */

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
acquisition_goal_type acquire_cache_site_fsm::acquisition_goal(void) const {
  if (ST_ACQUIRE_GOAL == current_state()) {
    return acquisition_goal_type::kCacheSite;
  }
  return acquisition_goal_type::kNone;
} /* acquisition_goal() */

NS_END(depth2, controller, fordyca);
