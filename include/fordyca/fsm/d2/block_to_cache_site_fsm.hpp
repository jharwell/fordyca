/**
 * \file block_to_cache_site_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_D2_BLOCK_TO_CACHE_SITE_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_D2_BLOCK_TO_CACHE_SITE_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/d2/acquire_cache_site_fsm.hpp"
#include "fordyca/fsm/acquire_free_block_fsm.hpp"
#include "fordyca/metrics/caches/site_selection_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, d2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_to_cache_site_fsm
 * \ingroup fsm d2
 *
 * \brief The FSM for the block-to-cache-site subtask.
 *
 * Each robot executing this FSM will locate a free block (either a known block
 * or via exploration), pickup the block and bring it to the location in the
 * arena that it has computed as being the best place to start a new cache, and
 * then drop the block there. Once it has done that it will signal that its task
 * is complete.
 */
class block_to_cache_site_fsm final : public block_to_goal_fsm,
                                      public virtual metrics::caches::site_selection_metrics {
 public:
  block_to_cache_site_fsm(const fsm_ro_params* c_params,
                          crfootbot::footbot_saa_subsystem* saa,
                          std::unique_ptr<csstrategy::base_strategy> exp_behavior,
                          rmath::rng* rng);

  ~block_to_cache_site_fsm(void) override = default;
  block_to_cache_site_fsm(const block_to_cache_site_fsm&) = delete;
  block_to_cache_site_fsm& operator=(const block_to_cache_site_fsm&) = delete;

  /* goal acquisition metrics */
  csmetrics::goal_acq_metrics::goal_type acquisition_goal(void) const override RCPPSW_PURE;
  rtypes::type_uuid entity_acquired_id(void) const override {
    return rtypes::constants::kNoUUID;
  }

  /* block transportation */
  foraging_transport_goal block_transport_goal(void) const override RCPPSW_PURE;
  bool is_phototaxiing_to_goal(bool) const override { return false; }

 private:
  /* clang-format off */
  acquire_cache_site_fsm m_cache_fsm;
  acquire_free_block_fsm m_block_fsm;
  /* clang-format on */

 public:
  /* cache site selection overrides */
  RCPPSW_WRAP_DECLDEF_OVERRIDE(site_select_exec, m_cache_fsm, const);
  RCPPSW_WRAP_DECLDEF_OVERRIDE(site_select_success, m_cache_fsm, const);
  RCPPSW_WRAP_DECLDEF_OVERRIDE(nlopt_result, m_cache_fsm, const);
};

NS_END(d2, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_D2_BLOCK_TO_CACHE_SITE_FSM_HPP_ */
