/**
 * @file block_to_goal_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH1_BLOCK_TO_GOAL_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH1_BLOCK_TO_GOAL_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/task_allocation/taskable.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/fsm/vector_fsm.hpp"
#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/acquire_block_fsm.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/fsm/block_transporter.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace task_allocation = rcppsw::task_allocation;
namespace visitor = rcppsw::patterns::visitor;
namespace representation { class perceived_arena_map; class block; }
NS_START(fsm);

using transport_goal_type = block_transporter::goal_type;

NS_START(depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_to_goal_fsm
 * @ingroup fsm depth1
 *
 * @brief Base FSM for acquiring, picking up a block, and then bringing it
 * somewhere and dropping it.
 *
 * Each robot executing this FSM will locate a free block (either a known block
 * or via random exploration), pickup the block and bring it to its chosen
 * goal. Once it has done that it will signal that its task is complete.
 */
class block_to_goal_fsm : public base_foraging_fsm,
                          public er::client<block_to_goal_fsm>,
                          public metrics::fsm::goal_acquisition_metrics,
                          public task_allocation::taskable,
                          public block_transporter,
                          public visitor::visitable_any<block_to_goal_fsm> {
 public:
  block_to_goal_fsm(const controller::block_selection_matrix* sel_matrix,
                    controller::saa_subsystem* saa,
                    representation::perceived_arena_map* map);

  block_to_goal_fsm(const block_to_goal_fsm& fsm) = delete;
  block_to_goal_fsm& operator=(const block_to_goal_fsm& fsm) = delete;

  /* taskable overrides */
  void task_execute(void) override;
  void task_start(const task_allocation::taskable_argument * arg) override;
  bool task_finished(void) const override { return ST_FINISHED == current_state(); }
  bool task_running(void) const override {
    return !(ST_FINISHED == current_state() || ST_START == current_state());
  }
  void task_reset(void) override { init(); }

  /* collision metrics */
  bool in_collision_avoidance(void) const override;
  bool entered_collision_avoidance(void) const override;
  bool exited_collision_avoidance(void) const override;
  uint collision_avoidance_duration(void) const override;

  /* goal acquisition metrics */
  bool is_exploring_for_goal(void) const override;
  bool is_vectoring_to_goal(void) const override;
  bool goal_acquired(void) const override;
  acquisition_goal_type acquisition_goal(void) const override;

  /**
   * @brief Reset the FSM
   */
  void init(void) override;

 protected:
  enum fsm_states {
    ST_START,
    /**
     * Superstate for acquiring a free block.
     */
    ST_ACQUIRE_FREE_BLOCK,

    /**
     * A block has been acquired--wait for area to send the block pickup signal.
     */
    ST_WAIT_FOR_BLOCK_PICKUP,

    /**
     * We are transporting a carried block to our goal.
     */
    ST_TRANSPORT_TO_GOAL,

    /**
     * We have acquired our goal--wait for arena to send the block drop signal.
     */
    ST_WAIT_FOR_BLOCK_DROP,

    /**
     * Block has been successfully dropped at our goal/in our goal.
     */
    ST_FINISHED,
    ST_MAX_STATES,
  };

  virtual acquire_goal_fsm& goal_fsm(void) = 0;
  const acquire_goal_fsm& goal_fsm(void) const {
    return const_cast<block_to_goal_fsm*>(this)->goal_fsm(); }
  const acquire_block_fsm& block_fsm(void) const { return m_block_fsm; }

 private:
  constexpr static uint kPICKUP_TIMEOUT = 100;

  /* inherited states */
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_wait_for_signal);

  /* block to goal states */
  HFSM_STATE_DECLARE(block_to_goal_fsm, start, state_machine::event_data);
  HFSM_STATE_DECLARE_ND(block_to_goal_fsm, acquire_free_block);
  HFSM_STATE_DECLARE(block_to_goal_fsm, wait_for_block_pickup,
                     state_machine::event_data);
  HFSM_STATE_DECLARE_ND(block_to_goal_fsm, transport_to_goal);
  HFSM_STATE_DECLARE(block_to_goal_fsm, wait_for_block_drop,
                     state_machine::event_data);
  HFSM_STATE_DECLARE_ND(block_to_goal_fsm, finished);

  /**
   * @brief Defines the state map for the FSM.
   *
   * Note that the order of the states in the map MUST match the order of the
   * states in \enum fsm_states, or things will not work correctly.
   */
  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
  return &mc_state_map[index];
  }

  // clang-format off
  uint              m_pickup_count;
  acquire_block_fsm m_block_fsm;
  // clang-format on

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(depth1, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH1_BLOCK_TO_GOAL_FSM_HPP_ */
