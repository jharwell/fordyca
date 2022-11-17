/**
 * \file acquire_new_cache_fsm.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "cosm/spatial/fsm/acquire_goal_fsm.hpp"
#include "cosm/subsystem/subsystem_fwd.hpp"

#include "fordyca/fsm/fsm_ro_params.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace controller::cognitive { class cache_sel_matrix; }

NS_START(fsm, d2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class acquire_new_cache_fsm
 * \ingroup fsm d2
 *
 * \brief The FSM for an acquiring a NEW cache within the arena.
 *
 * Each robot executing this FSM will look for a new cache (either a known new
 * cache or via exploration). Once the chosen new cache has been acquired, it
 * signals that it has completed its task.
 */
class acquire_new_cache_fsm final : public rer::client<acquire_new_cache_fsm>,
                              public csfsm::acquire_goal_fsm {
 public:
  acquire_new_cache_fsm(const fsm_ro_params* c_ro,
                        const csfsm::fsm_params* c_no,
                        std::unique_ptr<cssexplore::base_explore> explore,
                        rmath::rng* rng);
  ~acquire_new_cache_fsm(void) override = default;

  acquire_new_cache_fsm(const acquire_new_cache_fsm& fsm) = delete;
  acquire_new_cache_fsm& operator=(const acquire_new_cache_fsm& fsm) = delete;

 private:
  /*
   * See \ref acquire_goal_fsm for the purpose of these callbacks.
   */
  csmetrics::goal_acq_metrics::goal_type acquisition_goal_internal(void) const RCPPSW_CONST;
  boost::optional<acquire_goal_fsm::candidate_type> cache_select(void);
  bool candidates_exist(void) const RCPPSW_PURE;
  bool cache_acquired_cb(bool explore_result) const RCPPSW_PURE;

  /* clang-format off */
  const controller::cognitive::cache_sel_matrix* const mc_matrix;
  const fspds::dpo_store*      const                   mc_store;
  /* clang-format on */
};

NS_END(d2, fsm, fordyca);
