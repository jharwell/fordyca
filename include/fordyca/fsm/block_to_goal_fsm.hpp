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

#ifndef INCLUDE_FORDYCA_FSM_BLOCK_TO_GOAL_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_BLOCK_TO_GOAL_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/ta/taskable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

using acq_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using transport_goal_type = fsm::block_transporter::goal_type;

class acquire_goal_fsm;
class acquire_free_block_fsm;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_to_goal_fsm
 * @ingroup fordyca fsm depth1
 *
 * @brief Base FSM for acquiring, picking up a block, and then bringing it
 * somewhere and dropping it.
 *
 * Each robot executing this FSM will locate a free block (either a known block
 * or via random exploration), pickup the block and bring it to its chosen
 * goal. Once it has done that it will signal that its task is complete.
 */
class block_to_goal_fsm : public rer::client<block_to_goal_fsm>,
                          public base_foraging_fsm,
                          public rta::taskable,
                          public metrics::fsm::goal_acquisition_metrics,
                          public fsm::block_transporter {
 public:
  block_to_goal_fsm(acquire_goal_fsm* goal_fsm,
                    acquire_goal_fsm* block_fsm,
                    controller::saa_subsystem* saa);
  ~block_to_goal_fsm(void) override = default;

  block_to_goal_fsm(const block_to_goal_fsm&) = delete;
  block_to_goal_fsm& operator=(const block_to_goal_fsm&) = delete;

  /* taskable overrides */
  void task_execute(void) override;
  void task_start(const rta::taskable_argument* arg) override;
  bool task_finished(void) const override {
    return ekST_FINISHED == current_state();
  }
  bool task_running(void) const override {
    return !(ekST_FINISHED == current_state() || ekST_START == current_state());
  }
  void task_reset(void) override { init(); }

  /* collision metrics */
  FSM_OVERRIDE_DECL(bool, in_collision_avoidance, const final);
  FSM_OVERRIDE_DECL(bool, entered_collision_avoidance, const final);
  FSM_OVERRIDE_DECL(bool, exited_collision_avoidance, const final);
  FSM_OVERRIDE_DECL(uint, collision_avoidance_duration, const final);

  /* goal acquisition metrics */
  FSM_OVERRIDE_DECL(rmath::vector2u, acquisition_loc, const final);
  FSM_OVERRIDE_DECL(bool, is_vectoring_to_goal, const final);
  FSM_OVERRIDE_DECL(exp_status, is_exploring_for_goal, const final);
  FSM_OVERRIDE_DECL(bool, goal_acquired, const);
  FSM_OVERRIDE_DECL(acq_goal_type, acquisition_goal, const);
  FSM_OVERRIDE_DECL(rmath::vector2u, current_explore_loc, const final);
  FSM_OVERRIDE_DECL(rmath::vector2u, current_vector_loc, const final);

  /**
   * @brief Reset the FSM
   */
  void init(void) override final;

 protected:
  enum fsm_states {
    ekST_START,
    /**
     * Superstate for acquiring a block (free or from a cache).
     */
    ekST_ACQUIRE_BLOCK,

    /**
     * A block has been acquired--wait for area to send the block pickup signal.
     */
    ekST_WAIT_FOR_BLOCK_PICKUP,

    /**
     * We are transporting a carried block to our goal.
     */
    ekST_TRANSPORT_TO_GOAL,

    /**
     * We have acquired our goal--wait for arena to send the block drop signal.
     */
    ekST_WAIT_FOR_BLOCK_DROP,

    /**
     * Block has been successfully dropped at our goal/in our goal.
     */
    ekST_FINISHED,
    ekST_MAX_STATES,
  };

  const acquire_goal_fsm* goal_fsm(void) const { return m_goal_fsm; }
  const acquire_goal_fsm* block_fsm(void) const { return m_block_fsm; }

 private:
  /* inherited states */
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_wait_for_signal);

  /* block to goal states */
  HFSM_STATE_DECLARE(block_to_goal_fsm, start, rfsm::event_data);
  HFSM_STATE_DECLARE_ND(block_to_goal_fsm, acquire_block);
  HFSM_STATE_DECLARE(block_to_goal_fsm, wait_for_block_pickup, rfsm::event_data);
  HFSM_STATE_DECLARE_ND(block_to_goal_fsm, transport_to_goal);
  HFSM_STATE_DECLARE(block_to_goal_fsm, wait_for_block_drop, rfsm::event_data);
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

  /* clang-format off */
  acquire_goal_fsm* const  m_goal_fsm;
  acquire_goal_fsm * const m_block_fsm;
  /* clang-format on */

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ekST_MAX_STATES);
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_BLOCK_TO_GOAL_FSM_HPP_ */
