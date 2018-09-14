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
    const controller::cache_selection_matrix* csel_matrix,
    controller::saa_subsystem* saa,
    ds::perceived_arena_map* const map)
    : base_acquire_cache_fsm(csel_matrix, saa, map),
      ER_CLIENT_INIT("fordyca.fsm.depth2.acquire_new_cache") {}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
bool acquire_new_cache_fsm::select_cache_for_acquisition(
    argos::CVector2* const acquisition) {
  controller::depth2::new_cache_selector selector(sel_matrix());

  /* A "new" cache is the same as a single block  */
  representation::perceived_block best =
      selector.calc_best(map()->perceived_blocks(), base_sensors()->position());
  /*a
   * If this happens, all the blocks we know of are too close for us to vector
   * to.
   */
  if (nullptr == best.ent) {
    return false;
  }
  ER_INFO("Select new cache for acquisition: %d@(%u, %u) [utility=%f]",
          best.ent->id(),
          best.ent->discrete_loc().first,
          best.ent->discrete_loc().second,
          best.density.last_result());
  *acquisition = best.ent->real_loc();
  return true;
} /* select_cache() */

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
