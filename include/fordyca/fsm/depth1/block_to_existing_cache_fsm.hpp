/**
 * @file block_to_existing_cache_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH1_BLOCK_TO_EXISTING_CACHE_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH1_BLOCK_TO_EXISTING_CACHE_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/depth1/block_to_goal_fsm.hpp"
#include "fordyca/fsm/depth1/acquire_existing_cache_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_to_existing_cache_fsm
 * @ingroup fsm depth1
 *
 * @brief The FSM for the block-to-existing-cache subtask.
 *
 * Each robot executing this FSM will locate a free block (either a known block
 * or via random exploration), pickup the block and bring it to the best
 * existing cache it knows about. Once it has done that it will signal that its
 * task is complete.
 */
class block_to_existing_cache_fsm : public block_to_goal_fsm {
 public:
  block_to_existing_cache_fsm(const controller::block_selection_matrix* bsel_matrix,
                              const controller::cache_selection_matrix* csel_matrix,
                              controller::saa_subsystem* saa,
                              representation::perceived_arena_map* map);

  block_to_existing_cache_fsm(const block_to_existing_cache_fsm& fsm) = delete;
  block_to_existing_cache_fsm& operator=(const block_to_existing_cache_fsm& fsm) = delete;

  acquire_existing_cache_fsm& goal_fsm(void) override { return m_cache_fsm; }

  /* goal acquisition metrics */
  acquisition_goal_type acquisition_goal(void) const override;
  bool goal_acquired(void) const override;

  /* block transportation */
  transport_goal_type block_transport_goal(void) const override;

 private:
  // clang-format off
  acquire_existing_cache_fsm m_cache_fsm;
  // clang-format on
};

NS_END(depth1, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH1_BLOCK_TO_EXISTING_CACHE_FSM_HPP_ */
