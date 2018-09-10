/**
 * @file stateful_foraging_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH0_STATEFUL_FORAGING_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH0_STATEFUL_FORAGING_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "rcppsw/task_allocation/taskable.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/fsm/block_transporter.hpp"

#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/acquire_block_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation { class perceived_arena_map; }
namespace visitor = rcppsw::patterns::visitor;
namespace task_allocation = rcppsw::task_allocation;

NS_START(fsm, depth0);

using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using transport_goal_type = block_transporter::goal_type;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class stateful_foraging_fsm
 * @ingroup fsm depth0
 *
 * @brief The FSM for an unpartitioned foraging task. Each robot executing this
 * FSM will locate for a block (either a known block or via random exploration),
 * pickup the block and bring it all the way back to the nest.
 *
 * This FSM will only pickup free blocks. Once it has brought a block all the
 * way to the nest and dropped it in the nest, it will signal that its task is
 * complete.
 */
class stateful_foraging_fsm : public base_foraging_fsm,
                              er::client<stateful_foraging_fsm>,
                              public metrics::fsm::goal_acquisition_metrics,
                              public block_transporter,
                              public task_allocation::taskable,
                              public visitor::visitable_any<depth0::stateful_foraging_fsm> {
 public:
  stateful_foraging_fsm(const controller::block_selection_matrix* sel_matrix,
                        controller::saa_subsystem* saa,
                        representation::perceived_arena_map* map);

  /* taskable overrides */
  void task_reset(void) override { init(); }
  void task_start(const task_allocation::taskable_argument*) override {}
  void task_execute(void) override;
  bool task_finished(void) const override { return ST_FINISHED == current_state(); }
  bool task_running(void) const override { return m_task_running; }

  /* collision metrics */
  bool in_collision_avoidance(void) const override;
  bool entered_collision_avoidance(void) const override;
  bool exited_collision_avoidance(void) const override;
  uint collision_avoidance_duration(void) const override;

  /* goal acquisition metrics */
  FSM_WRAPPER_DECLARE(bool, is_exploring_for_goal);
  FSM_WRAPPER_DECLARE(bool, is_vectoring_to_goal);
  bool goal_acquired(void) const override;
  acquisition_goal_type acquisition_goal(void) const override;

  /* block transportation */
  transport_goal_type block_transport_goal(void) const override;

  /**
   * @brief Reset the FSM.
   */
  void init(void) override;

 protected:
  enum fsm_states {
    ST_START,
    ST_ACQUIRE_BLOCK,     /* superstate for finding a block */
    /**
     * @brief State robots wait in after acquiring a block for the simulation to
     * send them the block pickup signal. Having this extra state solves a lot
     * of handshaking/off by one issues regarding the timing of doing so.
     */
    ST_WAIT_FOR_PICKUP,
    ST_WAIT_FOR_DROP,
    ST_TRANSPORT_TO_NEST, /* take block to nest */
    ST_LEAVING_NEST,      /* Block dropped in nest--time to go */
    ST_FINISHED,
    ST_MAX_STATES
  };

 private:
  /* inherited states */
  HFSM_STATE_INHERIT(base_foraging_fsm, leaving_nest,
                     state_machine::event_data);
  HFSM_STATE_INHERIT(base_foraging_fsm, transport_to_nest,
                     state_machine::event_data);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_wait_for_signal);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_transport_to_nest);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_leaving_nest);

  /* depth0 foraging states */
  HFSM_STATE_DECLARE(stateful_foraging_fsm, start, state_machine::event_data);
  HFSM_STATE_DECLARE_ND(stateful_foraging_fsm, acquire_block);
  HFSM_STATE_DECLARE(stateful_foraging_fsm,
                     wait_for_pickup,
                     state_machine::event_data);
  HFSM_STATE_DECLARE(stateful_foraging_fsm,
                     wait_for_drop,
                     state_machine::event_data);
  HFSM_STATE_DECLARE_ND(stateful_foraging_fsm, finished);

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
  bool              m_task_running{false};
  acquire_block_fsm m_block_fsm;
  // clang-format on

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(depth0, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH0_STATEFUL_FORAGING_FSM_HPP_ */
