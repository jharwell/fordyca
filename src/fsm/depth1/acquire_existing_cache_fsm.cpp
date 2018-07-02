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
#include "fordyca/fsm/depth1/acquire_existing_cache_fsm.hpp"
#include "fordyca/controller/base_sensing_subsystem.hpp"
#include "fordyca/controller/depth1/existing_cache_selector.hpp"
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
acquire_existing_cache_fsm::acquire_existing_cache_fsm(
    const struct params::fsm_params* params,
    const std::shared_ptr<rcppsw::er::server>& server,
    controller::saa_subsystem* const saa,
    representation::perceived_arena_map* const map)
    : base_acquire_cache_fsm(params, server, saa, map) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool acquire_existing_cache_fsm::select_cache_for_acquisition(
    argos::CVector2* const acquisition) {
  controller::depth1::existing_cache_selector selector(server_ref(),
                                                       nest_center());
  representation::perceived_cache best =
      selector.calc_best(map()->perceived_caches(), base_sensors()->position());
  /*
   * If this happens, all the blocks we know of are too close for us to vector
   * to.
   */
  if (nullptr == best.ent) {
    return false;
  }
  ER_NOM("Select cache for acquisition: %d@(%zu, %zu) [utility=%f]",
         best.ent->id(),
         best.ent->discrete_loc().first,
         best.ent->discrete_loc().second,
         best.density.last_result());
  *acquisition = best.ent->real_loc();
  return true;
} /* select_cache_for_acquisition() */

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
acquisition_goal_type acquire_existing_cache_fsm::acquisition_goal(void) const {
  if (ST_ACQUIRE_GOAL == current_state()) {
    return acquisition_goal_type::kExistingCache;
  }
  return acquisition_goal_type::kNone;
} /* acquisition_goal() */

NS_END(depth1, controller, fordyca);
