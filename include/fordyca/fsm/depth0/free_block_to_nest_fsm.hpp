/**
 * @file free_block_to_nest_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH0_FREE_BLOCK_TO_NEST_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH0_FREE_BLOCK_TO_NEST_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/ta/taskable.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/fsm/block_transporter.hpp"

#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/acquire_free_block_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace ds { class dpo_semantic_map; }

NS_START(fsm, depth0);

using acq_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using transport_goal_type = block_transporter::goal_type;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class free_block_to_nest_fsm
 * @ingroup fordyca fsm depth0
 *
 * @brief FILL ME IN!
 */
class free_block_to_nest_fsm final : public base_foraging_fsm,
                                     public rer::client<free_block_to_nest_fsm>,
                                     public metrics::fsm::goal_acquisition_metrics,
                                     public block_transporter,
                                     public rta::taskable {
 public:
  free_block_to_nest_fsm(
      const controller::block_sel_matrix* sel_matrix,
      controller::saa_subsystem* saa,
      ds::dpo_store* store,
      std::unique_ptr<expstrat::base_expstrat> exp_behavior);

  /* taskable overrides */
  void task_execute(void) override;
  void task_reset(void) override { init(); }
  void task_start(const rta::taskable_argument*) override {}

  bool task_finished(void) const override {
    return ekST_FINISHED == current_state();
  }
  bool task_running(void) const override {
    return !(ekST_FINISHED == current_state() || ekST_START == current_state());
  }

  /* collision metrics */
  bool in_collision_avoidance(void) const override;
  bool entered_collision_avoidance(void) const override;
  bool exited_collision_avoidance(void) const override;
  uint collision_avoidance_duration(void) const override;

  /* goal acquisition metrics */
  FSM_OVERRIDE_DECL(exp_status, is_exploring_for_goal, const);
  FSM_OVERRIDE_DECL(bool, is_vectoring_to_goal, const);
  FSM_OVERRIDE_DECL(rmath::vector2u, acquisition_loc, const);
  FSM_OVERRIDE_DECL(rmath::vector2u, current_explore_loc, const);
  FSM_OVERRIDE_DECL(rmath::vector2u, current_vector_loc, const);

  bool goal_acquired(void) const override;
  acq_goal_type acquisition_goal(void) const override;

  /* block transportation */
  transport_goal_type block_transport_goal(void) const override;

  void init(void) override;

 protected:
  enum fsm_states {
    ekST_START,
    ekST_ACQUIRE_BLOCK,     /* superstate for finding a block */
    /**
     * @brief State robots wait in after acquiring a block for the simulation to
     * send them the block pickup signal. Having this extra state solves a lot
     * of handshaking/off by one issues regarding the timing of doing so.
     */
    ekST_WAIT_FOR_PICKUP,
    ekST_WAIT_FOR_DROP,
    ekST_TRANSPORT_TO_NEST, /* take block to nest */
    ekST_FINISHED,
    ekST_MAX_STATES
  };

 private:
  /* inherited states */
  HFSM_STATE_INHERIT(base_foraging_fsm, leaving_nest,
                     rfsm::event_data);
  HFSM_STATE_INHERIT(base_foraging_fsm, transport_to_nest,
                     rfsm::event_data);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_wait_for_signal);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_transport_to_nest);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_leaving_nest);
  HFSM_STATE_DECLARE(free_block_to_nest_fsm, start, rfsm::event_data);
  HFSM_STATE_DECLARE_ND(free_block_to_nest_fsm, acquire_block);
  HFSM_STATE_DECLARE(free_block_to_nest_fsm,
                     wait_for_pickup,
                     rfsm::event_data);

  /* depth0 foraging states */
  HFSM_STATE_DECLARE(free_block_to_nest_fsm,
                     wait_for_drop,
                     rfsm::event_data);

  HFSM_STATE_DECLARE_ND(free_block_to_nest_fsm, finished);

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
  acquire_free_block_fsm m_block_fsm;
  /* clang-format on */

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ekST_MAX_STATES);
};

NS_END(depth0, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH0_FREE_BLOCK_TO_NEST_FSM_HPP_ */
