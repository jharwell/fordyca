/**
 * \file acquire_cache_site_fsm.cpp
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
#include "fordyca/fsm/depth2/acquire_cache_site_fsm.hpp"

#include "fordyca/controller/cache_sel_matrix.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/fsm/arrival_tol.hpp"
#include "fordyca/fsm/depth2/cache_site_selector.hpp"
#include "fordyca/fsm/foraging_goal_type.hpp"

#include "cosm/repr/base_block2D.hpp"
#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth2);
using cselm = controller::cache_sel_matrix;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_cache_site_fsm::acquire_cache_site_fsm(const fsm_ro_params* c_params,
                                               crfootbot::footbot_saa_subsystem* saa,
                                               rmath::rng* rng)
    : ER_CLIENT_INIT("fordyca.fsm.depth2.acquire_cache_site"),
      acquire_goal_fsm(
          saa,
          nullptr, /* never explore for cache sites */
          rng,
          acquire_goal_fsm::hook_list{
            .acquisition_goal = std::bind(&acquire_cache_site_fsm::acquisition_goal_internal,
                                          this),
                .goal_select = std::bind(&acquire_cache_site_fsm::site_select, this),
                .candidates_exist = std::bind(&acquire_cache_site_fsm::candidates_exist,
                                              this),
                .begin_acq_cb = std::bind(&acquire_cache_site_fsm::reset_metrics, this),
                .goal_acquired_cb = std::bind(&acquire_cache_site_fsm::site_acquired_cb,
                                              this,
                                              std::placeholders::_1),
                .explore_term_cb = std::bind(&acquire_cache_site_fsm::site_exploration_term_cb,
                                             this),
                .goal_valid_cb = [](const rmath::vector2d&,
                                    const rtypes::type_uuid&) noexcept { return true;
}
}),
      mc_matrix(c_params->csel_matrix),
      mc_store(c_params->store) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool acquire_cache_site_fsm::site_acquired_cb(bool explore_result) const {
  ER_ASSERT(!explore_result, "Found cache site by exploring?");
  return true;
} /* site_acquired_cb() */

bool acquire_cache_site_fsm::site_exploration_term_cb(void) const {
  ER_FATAL_SENTINEL("Cache site acquired through exploration");
  return false;
} /* site_exploration_term_cb() */

boost::optional<cfsm::acquire_goal_fsm::candidate_type> acquire_cache_site_fsm::
    site_select(void) {
  auto selector = cache_site_selector(mc_matrix);
  if (auto best = selector(mc_store->caches(),
                           mc_store->blocks(),
                           saa()->sensing()->position(),
                           rng())) {
    ER_INFO("Select cache site@%s for acquisition", best->to_str().c_str());
    m_sel_success = true;
    m_sel_exec = true;
    m_nlopt_res = selector.nlopt_res();
    return boost::make_optional(
        acquire_goal_fsm::candidate_type(*best, kCACHE_SITE_ARRIVAL_TOL, -1));
  } else {
    ER_WARN("No cache site selected for acquisition--internal error?");
    m_sel_success = false;
    m_sel_exec = true;
    return boost::optional<acquire_goal_fsm::candidate_type>();
  }
} /* site_select() */

cfmetrics::goal_acq_metrics::goal_type acquire_cache_site_fsm::
    acquisition_goal_internal(void) const {
  return cfmetrics::goal_acq_metrics::goal_type(
      foraging_acq_goal::type::ekCACHE_SITE);
} /* acquisition_goal_internal() */

NS_END(depth2, controller, fordyca);
