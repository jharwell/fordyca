/**
 * \file block_to_existing_cache_fsm.cpp
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
#include "fordyca/fsm/acquire_existing_cache_fsm.hpp"

#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/robots/footbot/footbot_saa_subsystem2D.hpp"
#include "cosm/robots/footbot/footbot_sensing_subsystem.hpp"

#include "fordyca/ds/dpo_store.hpp"
#include "fordyca/fsm/arrival_tol.hpp"
#include "fordyca/fsm/cache_acq_point_selector.hpp"
#include "fordyca/fsm/cache_acq_validator.hpp"
#include "fordyca/fsm/existing_cache_selector.hpp"
#include "fordyca/fsm/expstrat/foraging_expstrat.hpp"
#include "fordyca/fsm/foraging_goal_type.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
using cselm = controller::cache_sel_matrix;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_existing_cache_fsm::acquire_existing_cache_fsm(
    const fsm_ro_params* c_params,
    crfootbot::footbot_saa_subsystem2D* saa,
    std::unique_ptr<fsm::expstrat::foraging_expstrat> exp_behavior,
    rmath::rng* rng,
    bool for_pickup)
    : ER_CLIENT_INIT("fordyca.fsm.acquire_existing_cache"),
      acquire_goal_fsm(
          saa,
          std::move(exp_behavior),
          rng,
          acquire_goal_fsm::hook_list{
              RCPPSW_STRUCT_DOT_INITIALIZER(
                  acquisition_goal,
                  std::bind(&acquire_existing_cache_fsm::acq_goal_internal)),
              RCPPSW_STRUCT_DOT_INITIALIZER(
                  goal_select,
                  std::bind(&acquire_existing_cache_fsm::existing_cache_select,
                            this)),
              RCPPSW_STRUCT_DOT_INITIALIZER(
                  candidates_exist,
                  std::bind(&acquire_existing_cache_fsm::candidates_exist, this)),

              RCPPSW_STRUCT_DOT_INITIALIZER(begin_acq_cb, nullptr),
              RCPPSW_STRUCT_DOT_INITIALIZER(
                  goal_acquired_cb,
                  std::bind(&acquire_existing_cache_fsm::cache_acquired_cb,

                            this,
                            std::placeholders::_1)),
              /*
                 * We never ever want to be able to acquire a cache via
                 * exploration, because if we are near/inside a cache exploring,
                 * it is because that cache is not currently suitable for
                 * acquisition (probably due to the current pickup
                 * policy). Allowing acquisition via exploration can result in
                 * violations of the pickup policy.
                 */
              RCPPSW_STRUCT_DOT_INITIALIZER(explore_term_cb,
                                            std::bind([]() noexcept {
                                              return false;
                                            })),
              RCPPSW_STRUCT_DOT_INITIALIZER(
                  goal_valid_cb,
                  std::bind(&acquire_existing_cache_fsm::cache_acq_valid,
                            this,
                            std::placeholders::_1,
                            std::placeholders::_2))}),
      mc_for_pickup(for_pickup),
      mc_matrix(c_params->csel_matrix),
      mc_store(c_params->store) {}

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
cfsm::metrics::goal_acq_metrics::goal_type acquire_existing_cache_fsm::
    acq_goal_internal(void) {
  return cfsm::metrics::goal_acq_metrics::goal_type(
      foraging_acq_goal::type::ekEXISTING_CACHE);
} /* acq_goal() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<acquire_existing_cache_fsm::acq_loc_type> acquire_existing_cache_fsm::
    calc_acq_location(void) {
  existing_cache_selector selector(mc_for_pickup, mc_matrix, &mc_store->caches());

  if (auto best = selector(mc_store->caches(),
                           saa()->sensing()->position(),
                           saa()->sensing()->tick())) {
    ER_INFO("Selected existing cache%d@%s/%s for acquisition",
            best->id().v(),
            best->rloc().to_str().c_str(),
            best->dloc().to_str().c_str());

    rmath::vector2d point = cache_acq_point_selector(
        kFOOTBOT_CACHE_ACQ_FACTOR)(saa()->sensing()->position(), best, rng());

    return boost::make_optional(std::make_pair(best->id(), point));
  } else {
    /*
     * If this happens, all the caches we know of are too close for us to vector
     * to, or otherwise unsuitable.
     */
    return boost::optional<acq_loc_type>();
  }
} /* calc_acq_location() */

boost::optional<cfsm::acquire_goal_fsm::candidate_type> acquire_existing_cache_fsm::
    existing_cache_select(void) {
  if (auto selection = calc_acq_location()) {
    return boost::make_optional(acquire_goal_fsm::candidate_type(
        selection.get().second, kCACHE_ARRIVAL_TOL, selection.get().first));
  }
  return boost::optional<acquire_goal_fsm::candidate_type>();
} /* existing_cache_select() */

bool acquire_existing_cache_fsm::candidates_exist(void) const {
  return !mc_store->caches().empty();
} /* candidates() */

bool acquire_existing_cache_fsm::cache_acquired_cb(bool explore_result) const {
  if (explore_result) {
    ER_FATAL_SENTINEL("Robot acquired cache via exploration");
    return false;
  } else {
    if (saa()->sensing()->sensor<chal::sensors::ground_sensor>()->detect(
            "cache")) {
      return true;
    }
    ER_WARN("Robot arrived at goal, but no cache was detected");
    return false;
  }
} /* cache_acquired_cb() */

bool acquire_existing_cache_fsm::cache_acq_valid(const rmath::vector2d& loc,
                                                 const rtypes::type_uuid& id) {
  return cache_acq_validator(&mc_store->caches(),
                             mc_matrix,
                             mc_for_pickup)(loc, id, saa()->sensing()->tick());
} /* cache_acq_valid() */

NS_END(controller, fordyca);
