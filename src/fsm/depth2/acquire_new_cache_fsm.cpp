/**
 * @file acquire_new_cache_fsm.cpp
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
#include "fordyca/fsm/depth2/acquire_new_cache_fsm.hpp"

#include "fordyca/controller/base_sensing_subsystem.hpp"
#include "fordyca/controller/depth2/new_cache_selector.hpp"
#include "fordyca/ds/perceived_arena_map.hpp"
#include "fordyca/representation/base_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth2);
namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_new_cache_fsm::acquire_new_cache_fsm(
    const controller::cache_sel_matrix* csel_matrix,
    controller::saa_subsystem* saa,
    ds::perceived_arena_map* const map)
    : acquire_goal_fsm(saa,
                       map,
                       std::bind([]() noexcept { return false; })),
      ER_CLIENT_INIT("fordyca.fsm.depth2.acquire_new_cache"),
      mc_sel_matrix(csel_matrix) {}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
bool acquire_new_cache_fsm::select_cache_for_acquisition(
    argos::CVector2* const acquisition) {
  controller::depth2::new_cache_selector selector(mc_sel_matrix);

  /* A "new" cache is the same as a single block  */
  representation::perceived_block best =
      selector.calc_best(map()->perceived_blocks(), base_sensors()->position());

  /*
   * If this happens, all the blocks we know of are ineligible for us to
   * vector to (too close or something similar).
   */
  if (nullptr == best.ent) {
    return false;
  }
  ER_INFO("Select new cache%d@(%f,%f) [%u, %u], utility=%f for acquisition",
          best.ent->id(),
          best.ent->real_loc().GetX(),
          best.ent->real_loc().GetY(),
          best.ent->discrete_loc().first,
          best.ent->discrete_loc().second,
          best.density.last_result());
  *acquisition = best.ent->real_loc();
  return true;
} /* select_cache_for_acquisition() */

bool acquire_new_cache_fsm::cache_acquired_cb(bool explore_result) const {
  ER_ASSERT(!explore_result, "New cache acquisition via exploration");
  argos::CVector2 position = saa_subsystem()->sensing()->position();
  for (auto &b : map()->blocks()) {
    if ((b->real_loc() - position).Length() <= vector_fsm::kCACHE_ARRIVAL_TOL) {
      return true;
    }
  } /* for(&b..) */
  ER_WARN("Robot arrived at location (%f,%f), but no known block within range.",
          position.GetX(),
          position.GetY());
  return false;
} /* cache_acquired_cb() */

bool acquire_new_cache_fsm::acquire_known_goal(void) {
  std::list<representation::perceived_block> blocks = map()->perceived_blocks();
  /*
   * If we don't know of any blocks and we are not current vectoring towards
   * one, then there is no way we can acquire a known block, so bail out.
   */
  if (blocks.empty() && !vector_fsm().task_running()) {
    return false;
  }

  if (!blocks.empty() && !vector_fsm().task_running()) {
    /*
     * If we get here, we must know of some blocks, but not be currently
     * vectoring toward any of them.
     */
    if (!vector_fsm().task_running()) {
      argos::CVector2 best;
      if (!select_cache_for_acquisition(&best)) {
        return false;
      }
      tasks::vector_argument v(vector_fsm::kCACHE_ARRIVAL_TOL, best);
      explore_fsm().task_reset();
      vector_fsm().task_reset();
      vector_fsm().task_start(&v);
    }
  }

  /* we are vectoring */
  if (!vector_fsm().task_finished()) {
    vector_fsm().task_execute();
  }

  if (vector_fsm().task_finished()) {
    vector_fsm().task_reset();
    return cache_acquired_cb(false);
  }
  return false;
} /* acquire_known_goal() */

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
acquisition_goal_type acquire_new_cache_fsm::acquisition_goal(void) const {
  if (ST_ACQUIRE_GOAL == current_state()) {
    return acquisition_goal_type::kNewCache;
  }
  return acquisition_goal_type::kNone;
} /* acquisition_goal() */

NS_END(depth2, controller, fordyca);
