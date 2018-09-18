/**
 * @file acquire_block_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_ACQUIRE_BLOCK_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_ACQUIRE_BLOCK_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>
#include <map>

#include "fordyca/controller/block_selection_matrix.hpp"
#include "fordyca/fsm/acquire_goal_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class acquire_block_fsm
 * @ingroup fsm
 *
 * @brief The FSM for an acquiring a free (i.e. not in a cache) block in the
 * arena.
 *
 * Each robot executing this FSM will look for a block (either a known block or
 * via stateless exploration). Once an existing block has been acquired, it
 * signals that it has completed its task.
 */
class acquire_block_fsm : public acquire_goal_fsm,
                          public er::client<acquire_block_fsm> {
 public:
  acquire_block_fsm(const controller::block_selection_matrix* matrix,
                    controller::saa_subsystem* saa,
                    ds::perceived_arena_map* map);

  acquire_block_fsm(const acquire_block_fsm& fsm) = delete;
  acquire_block_fsm& operator=(const acquire_block_fsm& fsm) = delete;

  /* goal acquisition metrics */
  acquisition_goal_type acquisition_goal(void) const override;

 private:
  bool acquire_known_goal(void) override;

  /**
   * @brief Callback for block detection used by parent class.
   *
   * @return \c TRUE if a block has been detected, \c FALSE otherwise.
   */
  bool block_detected_cb(void) const;

  /**
   * @brief Callback used after a block has been acquired for sanity
   * check/verification of state.
   *
   * @param explore_result If \c TRUE, then the acquired block was obtained via
   * exploration, rather than vectoring.
   *
   * @return \c TRUE if a block REALLY has been acquired, and \c FALSE
   * otherwise.
   */
  bool block_acquired_cb(bool explore_result) const;

  // clang-format off
  const controller::block_selection_matrix* const mc_matrix;
  // clang-format on
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_ACQUIRE_BLOCK_FSM_HPP_ */
