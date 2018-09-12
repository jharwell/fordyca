/**
 * @file base_acquire_cache_fsm.cpp
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
#include "fordyca/fsm/depth1/base_acquire_cache_fsm.hpp"

#include "fordyca/controller/depth1/sensing_subsystem.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth1);
namespace depth1 = controller::depth1;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_acquire_cache_fsm::base_acquire_cache_fsm(
    const controller::cache_selection_matrix* sel_matrix,
    controller::saa_subsystem* const saa,
    representation::perceived_arena_map* const map)
    : acquire_goal_fsm(saa,
                       map,
                       std::bind(&base_acquire_cache_fsm::cache_detected_cb,
                                 this)),
      ER_CLIENT_INIT("fordyca.fsm.depth1.base_acquire_cache"),
      mc_sel_matrix(sel_matrix) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool base_acquire_cache_fsm::cache_detected_cb(void) const {
  const auto& sensors =
      std::static_pointer_cast<const depth1::sensing_subsystem>(base_sensors());
  return sensors->cache_detected();
} /* block_detected_cb() */

bool base_acquire_cache_fsm::cache_acquired_cb(bool explore_result) const {
  const auto& sensors =
      std::static_pointer_cast<const depth1::sensing_subsystem>(base_sensors());
  if (explore_result) {
    ER_ASSERT(sensors->cache_detected(),
              "No cache detected after successful exploration?");
    return true;
  } else {
    if (sensors->cache_detected()) {
      return true;
    }
    ER_WARN("Robot arrived at goal, but no cache was detected.");
    return false;
  }
} /* cache_acquired_cb() */

bool base_acquire_cache_fsm::acquire_known_goal(void) {
  std::list<representation::perceived_cache> caches = map()->perceived_caches();
  /*
   * If we don't know of any caches and we are not current vectoring towards
   * one, then there is no way we can acquire a known cache, so bail out.
   */
  if (caches.empty() && !vector_fsm().task_running()) {
    return false;
  }

  if (!caches.empty() && !vector_fsm().task_running()) {
    /*
     * If we get here, we must know of some caches, but not be currently
     * vectoring toward any of them.
     */
    if (!vector_fsm().task_running()) {
      argos::CVector2 best;
      /*
     * If this happens, all the blocks we know of are too close for us to vector
     * to.
     */
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

NS_END(depth1, controller, fordyca);
