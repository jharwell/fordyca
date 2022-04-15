/**
 * \file acquire_new_cache_fsm.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/fsm/d2/acquire_new_cache_fsm.hpp"

#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "fordyca/controller/cognitive/cache_sel_matrix.hpp"
#include "fordyca/controller/cognitive/d2/new_cache_selector.hpp"
#include "fordyca/fsm/arrival_tol.hpp"
#include "fordyca/fsm/foraging_acq_goal.hpp"
#include "fordyca/subsystem/perception/ds/dpo_semantic_map.hpp"
#include "fordyca/subsystem/perception/ds/dpo_store.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, d2);
using cselm = controller::cognitive::cache_sel_matrix;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_new_cache_fsm::acquire_new_cache_fsm(
    const fsm_ro_params* c_ro,
    const csfsm::fsm_params* c_no,
    std::unique_ptr<cssexplore::base_explore> explore,
    rmath::rng* rng)
    : ER_CLIENT_INIT("fordyca.fsm.d2.acquire_new_cache"),
      acquire_goal_fsm(
          c_no,
          std::move(explore),
          rng,
          acquire_goal_fsm::hook_list{
              RCPPSW_STRUCT_DOT_INITIALIZER(
                  acquisition_goal,
                  std::bind(&acquire_new_cache_fsm::acquisition_goal_internal,
                            this)),
              RCPPSW_STRUCT_DOT_INITIALIZER(
                  goal_select,
                  std::bind(&acquire_new_cache_fsm::cache_select, this)),
              RCPPSW_STRUCT_DOT_INITIALIZER(
                  candidates_exist,
                  std::bind(&acquire_new_cache_fsm::candidates_exist, this)),
              RCPPSW_STRUCT_DOT_INITIALIZER(begin_acq_cb, nullptr),
              RCPPSW_STRUCT_DOT_INITIALIZER(
                  goal_acquired_cb,
                  std::bind(&acquire_new_cache_fsm::cache_acquired_cb,
                            this,
                            std::placeholders::_1)),

              /* new caches never acquired via exploration */
              RCPPSW_STRUCT_DOT_INITIALIZER(
                  explore_term_cb,
                  std::bind([](void) noexcept { return false; })),
              RCPPSW_STRUCT_DOT_INITIALIZER(
                  goal_valid_cb,
                  [](const rmath::vector2d&, const rtypes::type_uuid&) noexcept {
                    return true;
                  }) }),
      mc_matrix(c_ro->csel_matrix),
      mc_store(c_ro->store) {}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
bool acquire_new_cache_fsm::candidates_exist(void) const {
  return !mc_store->known_blocks().empty();
} /* candidates_exsti() */

boost::optional<csfsm::acquire_goal_fsm::candidate_type>
acquire_new_cache_fsm::cache_select(void) {
  controller::cognitive::d2::new_cache_selector selector(mc_matrix);

  /* A "new" cache is the same as a single block  */
  if (const auto* best = selector(mc_store->tracked_blocks(),
                                  mc_store->tracked_caches(),
                                  sensing()->rpos2D())) {
    ER_INFO("Select new cache%d@%s/%s for acquisition",
            best->id().v(),
            rcppsw::to_string(best->ranchor2D()).c_str(),
            rcppsw::to_string(best->danchor2D()).c_str());

    auto tol = std::get<rtypes::spatial_dist>(
        mc_matrix->find(cselm::kNewCacheDropTolerance)->second);
    return boost::make_optional(
        acquire_goal_fsm::candidate_type(best->rcenter2D(), tol.v(), best->id()));
  } else {
    /*
     * If this happens, all the blocks we know of are ineligible for us to
     * vector to (too close or something similar).
     */
    return boost::optional<acquire_goal_fsm::candidate_type>();
  }
} /* cache_select() */

bool acquire_new_cache_fsm::cache_acquired_cb(bool explore_result) const {
  ER_ASSERT(!explore_result, "New cache acquisition via exploration?");
  rmath::vector2d position = saa()->sensing()->rpos2D();
  for (const auto& b : mc_store->known_blocks()) {
    if ((b->rcenter2D() - position).length() <= kNEW_CACHE_ARRIVAL_TOL) {
      return true;
    }
  } /* for(&b..) */
  ER_WARN("Robot arrived at location %s, but no known block within range.",
          position.to_str().c_str());
  return false;
} /* cache_acquired_cb() */

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
csmetrics::goal_acq_metrics::goal_type
acquire_new_cache_fsm::acquisition_goal_internal(void) const {
  return fsm::to_goal_type(foraging_acq_goal::ekNEW_CACHE);
} /* acquisition_goal() */

NS_END(d2, controller, fordyca);
