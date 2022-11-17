/**
 * \file block_to_existing_cache_fsm.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/acquire_existing_cache_fsm.hpp"

#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "fordyca/fsm/arrival_tol.hpp"
#include "fordyca/fsm/cache_acq_point_selector.hpp"
#include "fordyca/fsm/cache_acq_validator.hpp"
#include "fordyca/fsm/existing_cache_selector.hpp"
#include "fordyca/fsm/foraging_acq_goal.hpp"
#include "fordyca/fsm/foraging_transport_goal.hpp"
#include "fordyca/subsystem/perception/ds/dpo_store.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_existing_cache_fsm::acquire_existing_cache_fsm(
    const fsm_ro_params* c_ro,
    const csfsm::fsm_params* c_no,
    std::unique_ptr<cssexplore::base_explore> explore,
    rmath::rng* rng,
    bool for_pickup)
    : ER_CLIENT_INIT("fordyca.fsm.acquire_existing_cache"),
      acquire_goal_fsm(
          c_no,
          std::move(explore),
          rng,
          acquire_goal_fsm::hook_list{

                  std::bind(&acquire_existing_cache_fsm::acq_goal_internal),

                  std::bind(&acquire_existing_cache_fsm::existing_cache_select,
                            this),

                  std::bind(&acquire_existing_cache_fsm::candidates_exist, this),

               nullptr,

                  std::bind(&acquire_existing_cache_fsm::cache_acquired_cb,

                            this,
                            std::placeholders::_1),
              /*
               * We never ever want to be able to acquire a cache via
               * exploration, because if we are near/inside a cache exploring,
               * it is because that cache is not currently suitable for
               * acquisition (probably due to the current pickup
               * policy). Allowing acquisition via exploration can result in
               * violations of the pickup policy.
               */

                  std::bind([]() noexcept { return false; }),

                  std::bind(&acquire_existing_cache_fsm::cache_acq_valid,
                            this,
                            std::placeholders::_1,
                            std::placeholders::_2)
          }),
      mc_for_pickup(for_pickup),
      mc_matrix(c_ro->csel_matrix),
      mc_store(c_ro->store) {}

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
csmetrics::goal_acq_metrics::goal_type
acquire_existing_cache_fsm::acq_goal_internal(void) {
  return fsm::to_goal_type(foraging_acq_goal::ekEXISTING_CACHE);
} /* acq_goal() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<acquire_existing_cache_fsm::acq_loc_type>
acquire_existing_cache_fsm::calc_acq_location(void) {
  existing_cache_selector selector(
      mc_for_pickup, mc_matrix, &mc_store->tracked_caches());

  if (const auto* best = selector(mc_store->tracked_caches(),
                                  saa()->sensing()->rpos2D(),
                                  saa()->sensing()->tick())) {
    ER_INFO("Selected existing cache%d@%s/%s for acquisition",
            best->id().v(),
            rcppsw::to_string(best->rcenter2D()).c_str(),
            rcppsw::to_string(best->dcenter2D()).c_str());

    rmath::vector2d point = cache_acq_point_selector(
        kFOOTBOT_CACHE_ACQ_FACTOR)(saa()->sensing()->rpos2D(), best, rng());

    return boost::make_optional(std::make_pair(best->id(), point));
  } else {
    /*
     * If this happens, all the caches we know of are too close for us to vector
     * to, or otherwise unsuitable.
     */
    return boost::optional<acq_loc_type>();
  }
} /* calc_acq_location() */

boost::optional<csfsm::acquire_goal_fsm::candidate_type>
acquire_existing_cache_fsm::existing_cache_select(void) {
  if (auto selection = calc_acq_location()) {
    return boost::make_optional(acquire_goal_fsm::candidate_type(
        selection.get().second, kCACHE_ARRIVAL_TOL, selection.get().first));
  }
  return boost::optional<acquire_goal_fsm::candidate_type>();
} /* existing_cache_select() */

bool acquire_existing_cache_fsm::candidates_exist(void) const {
  return !mc_store->known_caches().empty();
} /* candidates() */

bool acquire_existing_cache_fsm::cache_acquired_cb(bool explore_result) {
  if (explore_result) {
    ER_FATAL_SENTINEL("Robot acquired cache via exploration");
    return false;
  } else {
    if (saa()->sensing()->env()->detect("cache")) {
      return true;
    }
    ER_WARN("Robot arrived at goal, but no cache was detected");
    return false;
  }
} /* cache_acquired_cb() */

bool acquire_existing_cache_fsm::cache_acq_valid(const rmath::vector2d& loc,
                                                 const rtypes::type_uuid& id) {
  auto caches = mc_store->known_caches();
  return cache_acq_validator(caches, mc_matrix, mc_for_pickup)(
      loc, id, saa()->sensing()->tick());
} /* cache_acq_valid() */

NS_END(controller, fordyca);
