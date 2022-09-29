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
#include "fordyca/fsm/d2/acquire_cache_site_fsm.hpp"

#include "cosm/repr/base_block3D.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

#include "fordyca/controller/cognitive/cache_sel_matrix.hpp"
#include "fordyca/fsm/arrival_tol.hpp"
#include "fordyca/fsm/d2/cache_site_selector.hpp"
#include "fordyca/fsm/foraging_acq_goal.hpp"
#include "fordyca/subsystem/perception/ds/dpo_semantic_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, d2);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_cache_site_fsm::acquire_cache_site_fsm(const fsm_ro_params* c_ro,
                                               const csfsm::fsm_params* c_no,
                                               rmath::rng* rng)
    : ER_CLIENT_INIT("fordyca.fsm.d2.acquire_cache_site"),
      acquire_goal_fsm(
          c_no,
          nullptr, /* never explore for cache sites */
          rng,
          acquire_goal_fsm::hook_list{

                  std::bind(&acquire_cache_site_fsm::acquisition_goal_internal,
                            this),

                  std::bind(&acquire_cache_site_fsm::site_select, this),

                  std::bind(&acquire_cache_site_fsm::candidates_exist, this),

                  std::bind(&acquire_cache_site_fsm::reset_metrics, this),

                  std::bind(&acquire_cache_site_fsm::site_acquired_cb,
                            this,
                            std::placeholders::_1),

                  std::bind(&acquire_cache_site_fsm::site_exploration_term_cb,
                            this),

                  [](const rmath::vector2d&, const rtypes::type_uuid&) noexcept {
                    return true;
                  } }),
      mc_matrix(c_ro->csel_matrix),
      mc_accessor(c_ro->accessor) {}

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

boost::optional<csfsm::acquire_goal_fsm::candidate_type>
acquire_cache_site_fsm::site_select(void) {
  auto selector = cache_site_selector(mc_matrix);
  if (auto best = selector(
          mc_accessor->known_caches(), saa()->sensing()->rpos2D(), rng())) {
    ER_INFO("Select cache site@%s for acquisition", best->to_str().c_str());
    m_sel_success = true;
    m_sel_exec = true;
    m_nlopt_res = selector.nlopt_res();
    return boost::make_optional(
        acquire_goal_fsm::candidate_type(*best, kCACHE_SITE_ARRIVAL_TOL, -1));
  } else {
    ER_WARN("No cache site selected for acquisition--possible internal error");
    m_sel_success = false;
    m_sel_exec = true;
    return boost::optional<acquire_goal_fsm::candidate_type>();
  }
} /* site_select() */

csmetrics::goal_acq_metrics::goal_type
acquire_cache_site_fsm::acquisition_goal_internal(void) const {
  return fsm::to_goal_type(foraging_acq_goal::ekCACHE_SITE);
} /* acquisition_goal_internal() */

NS_END(d2, controller, fordyca);
