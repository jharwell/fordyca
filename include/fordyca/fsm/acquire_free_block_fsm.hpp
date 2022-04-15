/**
 * \file acquire_free_block_fsm.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "cosm/spatial/fsm/acquire_goal_fsm.hpp"
#include "cosm/subsystem/subsystem_fwd.hpp"
#include "cosm/ta/taskable.hpp"

#include "fordyca/controller/cognitive/block_sel_matrix.hpp"
#include "fordyca/fsm/foraging_acq_goal.hpp"
#include "fordyca/fsm/foraging_transport_goal.hpp"
#include "fordyca/fsm/fsm_ro_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class acquire_free_block_fsm
 * \ingroup fsm
 *
 * \brief The FSM for an acquiring a free (i.e. not in a cache) block in the
 * arena.
 *
 * Each robot executing this FSM will look for a block (either a known block or
 * via stateless exploration). Once an existing block has been acquired, it
 * signals that it has completed its task.
 */
class acquire_free_block_fsm : public rer::client<acquire_free_block_fsm>,
                               public csfsm::acquire_goal_fsm {
 public:
  acquire_free_block_fsm(const fsm_ro_params* c_ro,
                         const csfsm::fsm_params* c_no,
                         std::unique_ptr<cssexplore::base_explore> behavior,
                         rmath::rng* rng);

  ~acquire_free_block_fsm(void) override = default;

  acquire_free_block_fsm(const acquire_free_block_fsm&) = delete;
  acquire_free_block_fsm& operator=(const acquire_free_block_fsm&) = delete;

 private:
  /*
   * See \ref acquire_goal_fsm for the purpose of these callbacks.
   */
  static csmetrics::goal_acq_metrics::goal_type
  acq_goal_internal(void) RCPPSW_CONST;
  boost::optional<acquire_goal_fsm::candidate_type> block_select(void) const;
  bool candidates_exist(void) const RCPPSW_PURE;
  bool block_exploration_term_cb(void);
  bool block_acquired_cb(bool explore_result);
  bool block_acq_valid(const rmath::vector2d& loc,
                       const rtypes::type_uuid& id) const;

  /* clang-format off */
  const controller::cognitive::block_sel_matrix* const mc_matrix;
  const fspds::dpo_store*      const                   mc_store;
  /* clang-format on */
};

NS_END(fsm, fordyca);
