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
#include "fordyca/fsm/depth2/acquire_new_cache_fsm.hpp"

#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"

#include "fordyca/controller/depth2/new_cache_selector.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/fsm/arrival_tol.hpp"
#include "fordyca/fsm/expstrat/foraging_expstrat.hpp"
#include "fordyca/fsm/foraging_goal_type.hpp"
#include "fordyca/repr/base_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth2);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_new_cache_fsm::acquire_new_cache_fsm(
    const fsm_ro_params* c_params,
    crfootbot::footbot_saa_subsystem* saa,
    std::unique_ptr<expstrat::foraging_expstrat> exp_behavior,
    rmath::rng* rng)
    : ER_CLIENT_INIT("fordyca.fsm.depth2.acquire_new_cache"),
      acquire_goal_fsm(
          saa,
          std::move(exp_behavior),
          rng,
          acquire_goal_fsm::hook_list{
              .acquisition_goal =
                  std::bind(&acquire_new_cache_fsm::acquisition_goal_internal,
                            this),
              .goal_select =
                  std::bind(&acquire_new_cache_fsm::cache_select, this),
              .candidates_exist =
                  std::bind(&acquire_new_cache_fsm::candidates_exist, this),
              .goal_acquired_cb =
                  std::bind(&acquire_new_cache_fsm::cache_acquired_cb,
                            this,
                            std::placeholders::_1),

              /* new caches never acquired via exploration */
              .explore_term_cb = std::bind([](void) noexcept { return false; }),
              .goal_valid_cb = [](const rmath::vector2d&,
                                  const rtypes::type_uuid&) { return true; }}),
      mc_matrix(c_params->csel_matrix),
      mc_store(c_params->store) {}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
bool acquire_new_cache_fsm::candidates_exist(void) const {
  return !mc_store->blocks().empty();
} /* candidates_exsti() */

boost::optional<cfsm::acquire_goal_fsm::candidate_type> acquire_new_cache_fsm::
    cache_select(void) const {
  controller::depth2::new_cache_selector selector(mc_matrix);

  /* A "new" cache is the same as a single block  */
  if (auto best = selector(
          mc_store->blocks(), mc_store->caches(), sensing()->position())) {
    ER_INFO("Select new cache%d@%s/%s,density=%f for acquisition",
            best->ent()->id().v(),
            best->ent()->rloc().to_str().c_str(),
            best->ent()->dloc().to_str().c_str(),
            best->density().v());
    return boost::make_optional(acquire_goal_fsm::candidate_type(
        best->ent()->rloc(), kNEW_CACHE_ARRIVAL_TOL, best->ent()->id()));
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
  rmath::vector2d position = saa()->sensing()->position();
  for (auto& b : mc_store->blocks().const_values_range()) {
    if ((b.ent()->rloc() - position).length() <= kNEW_CACHE_ARRIVAL_TOL) {
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
cfmetrics::goal_acq_metrics::goal_type acquire_new_cache_fsm::
    acquisition_goal_internal(void) const {
  return cfmetrics::goal_acq_metrics::goal_type(
      foraging_acq_goal::type::ekNEW_CACHE);
} /* acquisition_goal() */

NS_END(depth2, controller, fordyca);
