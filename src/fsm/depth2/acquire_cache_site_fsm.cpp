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

#include "fordyca/controller/cache_sel_matrix.hpp"
#include "fordyca/controller/depth1/sensing_subsystem.hpp"
#include "fordyca/controller/depth2/cache_site_selector.hpp"
#include "fordyca/ds/perceived_arena_map.hpp"
#include "fordyca/representation/base_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth2);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_cache_site_fsm::acquire_cache_site_fsm(
    const controller::cache_sel_matrix* csel_matrix,
    controller::saa_subsystem* const saa,
    ds::perceived_arena_map* const map)
    : acquire_goal_fsm(saa,
                       map,
                       []() { return false; }), /* We should never acquire a
                                                 * cache site by exploring */
      ER_CLIENT_INIT("fordyca.fsm.depth2.acquire_cache_site"),
      mc_matrix(csel_matrix) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
__rcsw_const bool acquire_cache_site_fsm::site_acquired_cb(
    bool explore_result) const {
  ER_ASSERT(!explore_result, "Found cache site by exploring?");
  argos::CVector2 robot_loc = saa_subsystem()->sensing()->position();
  for (auto& b : map()->blocks()) {
    if ((robot_loc - b->real_loc()).Length() <=
        boost::get<double>(mc_matrix->find("block_prox_dist")->second)) {
      ER_WARN("Cannot acquire cache site@(%f,%f): Block%d@(%f,%f) too close",
              robot_loc.GetX(),
              robot_loc.GetY(),
              b->id(),
              b->real_loc().GetX(),
              b->real_loc().GetY());
      return false;
    }
  } /* for(&b..) */

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
    controller::depth2::cache_site_selector s(mc_matrix);
    tasks::vector_argument v(vector_fsm::kCACHE_SITE_ARRIVAL_TOL,
                             s.calc_best(map()->caches(),
                                         map()->blocks(),
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
