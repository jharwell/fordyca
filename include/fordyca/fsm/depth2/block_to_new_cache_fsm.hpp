/**
 * @file block_to_new_cache_fsm.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH2_BLOCK_TO_NEW_CACHE_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH2_BLOCK_TO_NEW_CACHE_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/acquire_free_block_fsm.hpp"
#include "fordyca/fsm/depth2/acquire_new_cache_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_to_new_cache_fsm
 * @ingroup fordyca fsm depth2
 *
 * @brief The FSM for the block-to-new-cache subtask.
 *
 * Each robot executing this FSM will locate a free block (either a known block
 * or via exploration), pickup the block and bring it to the best new cache it
 * knows about. Once it has done that it will signal that its task is complete.
 */
class block_to_new_cache_fsm final : public block_to_goal_fsm {
 public:
  block_to_new_cache_fsm(
      const controller::block_sel_matrix* bsel_matrix,
      const controller::cache_sel_matrix* csel_matrix,
      controller::saa_subsystem* saa,
      ds::dpo_store* store,
      std::unique_ptr<expstrat::base_expstrat> exp_behavior);
  ~block_to_new_cache_fsm(void) override = default;

  block_to_new_cache_fsm(const block_to_new_cache_fsm& fsm) = delete;
  block_to_new_cache_fsm& operator=(const block_to_new_cache_fsm& fsm) = delete;

  /* goal acquisition metrics */
  acquisition_goal_type acquisition_goal(void) const override;

  /* block transportation */
  transport_goal_type block_transport_goal(void) const override;

  /* clang-format off */
  acquire_new_cache_fsm  m_cache_fsm;
  acquire_free_block_fsm m_block_fsm;
  /* clang-format on */
};

NS_END(depth2, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH2_BLOCK_TO_NEW_CACHE_FSM_HPP_ */
