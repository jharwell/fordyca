/**
 * @file acquire_free_block_fsm.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_FSM_ACQUIRE_FREE_BLOCK_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_ACQUIRE_FREE_BLOCK_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/block_sel_matrix.hpp"
#include "fordyca/fsm/acquire_goal_fsm.hpp"
#include "fordyca/metrics/fsm/collision_metrics.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "rcppsw/task_allocation/taskable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace ds {
class dpo_store;
}
using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
namespace ta = rcppsw::task_allocation;

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class acquire_free_block_fsm
 * @ingroup fsm
 *
 * @brief The FSM for an acquiring a free (i.e. not in a cache) block in the
 * arena.
 *
 * Each robot executing this FSM will look for a block (either a known block or
 * via stateless exploration). Once an existing block has been acquired, it
 * signals that it has completed its task.
 */
class acquire_free_block_fsm : public er::client<acquire_free_block_fsm>,
                               public acquire_goal_fsm {
 public:
  acquire_free_block_fsm(const controller::block_sel_matrix* matrix,
                         controller::saa_subsystem* saa,
                         ds::dpo_store* store);

  ~acquire_free_block_fsm(void) override = default;

  acquire_free_block_fsm(const acquire_free_block_fsm& fsm) = delete;
  acquire_free_block_fsm& operator=(const acquire_free_block_fsm& fsm) = delete;

 private:
  /*
   * See \ref acquire_goal_fsm for the purpose of these callbacks.
   */
  acquisition_goal_type acquisition_goal_internal(void) const;
  acquire_goal_fsm::candidate_type block_select(void) const;
  bool candidates_exist(void) const;
  bool block_exploration_term_cb(void) const;
  bool block_acquired_cb(bool explore_result) const;

  /* clang-format off */
  const controller::block_sel_matrix* const mc_matrix;
  const ds::dpo_store*      const           mc_store;
  /* clang-format on */
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_ACQUIRE_FREE_BLOCK_FSM_HPP_ */
