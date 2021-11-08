/**
 * \file acquire_new_cache_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_D2_ACQUIRE_NEW_CACHE_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_D2_ACQUIRE_NEW_CACHE_FSM_HPP_

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
                        std::unique_ptr<csstrategy::base_strategy> exp_behavior,
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

#endif /* INCLUDE_FORDYCA_FSM_D2_ACQUIRE_NEW_CACHE_FSM_HPP_ */
