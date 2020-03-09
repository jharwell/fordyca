/**
 * \file free_block_to_nest_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH0_FREE_BLOCK_TO_NEST_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH0_FREE_BLOCK_TO_NEST_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "cosm/ta/taskable.hpp"
#include "fordyca/fsm/block_transporter.hpp"
#include "cosm/fsm/metrics/goal_acq_metrics.hpp"
#include "fordyca/fsm/fsm_ro_params.hpp"

#include "cosm/fsm/util_hfsm.hpp"
#include "fordyca/fsm/acquire_free_block_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace ds { class dpo_semantic_map; }

namespace fsm::expstrat {
class foraging_expstrat;
} /* namespace fsm::expstrat */

NS_START(fsm, depth0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class free_block_to_nest_fsm
 * \ingroup fsm depth0
 *
 * \brief FILL ME IN!
 */
class free_block_to_nest_fsm final : public cfsm::util_hfsm,
                                     public rer::client<free_block_to_nest_fsm>,
                                     public cfsm::metrics::goal_acq_metrics,
                                     public block_transporter,
                                     public cta::taskable {
 public:
  free_block_to_nest_fsm(
      const fsm_ro_params* c_params,
      crfootbot::footbot_saa_subsystem2D* saa,
      std::unique_ptr<fsm::expstrat::foraging_expstrat> exp_behavior,
      rmath::rng* rng);

  free_block_to_nest_fsm(const free_block_to_nest_fsm&) = delete;
  free_block_to_nest_fsm& operator=(const free_block_to_nest_fsm&) = delete;

  /* taskable overrides */
  void task_execute(void) override;
  void task_reset(void) override { init(); }
  void task_start(const cta::taskable_argument*) override {}

  bool task_finished(void) const override {
    return ekST_FINISHED == current_state();
  }
  bool task_running(void) const override {
    return !(ekST_FINISHED == current_state() || ekST_START == current_state());
  }

  /* collision metrics */
  bool in_collision_avoidance(void) const override RCSW_PURE;
  bool entered_collision_avoidance(void) const override RCSW_PURE;
  bool exited_collision_avoidance(void) const override RCSW_PURE;
  rtypes::timestep collision_avoidance_duration(void) const override RCSW_PURE;
  rmath::vector2u avoidance_loc(void) const override RCSW_PURE;

  /* goal acquisition metrics */
  RCPPSW_WRAP_OVERRIDE_DECL(exp_status, is_exploring_for_goal, const);
  RCPPSW_WRAP_OVERRIDE_DECL(bool, is_vectoring_to_goal, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector2u, acquisition_loc, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector2u, current_explore_loc, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector2u, current_vector_loc, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rtypes::type_uuid, entity_acquired_id, const);

  bool goal_acquired(void) const override RCSW_PURE;
  cfsm::metrics::goal_acq_metrics::goal_type acquisition_goal(void) const override RCSW_PURE;

  /* block transportation */
  fsm::foraging_transport_goal::type block_transport_goal(void) const override RCSW_PURE;

  void init(void) override;

 protected:
  enum fsm_states {
    ekST_START,
    ekST_ACQUIRE_BLOCK,     /* superstate for finding a block */
    /**
     * \brief State robots wait in after acquiring a block for the simulation to
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
  HFSM_STATE_INHERIT(cfsm::util_hfsm, leaving_nest,
                     rpfsm::event_data);
  HFSM_STATE_INHERIT(cfsm::util_hfsm, transport_to_nest,
                     rpfsm::event_data);
  HFSM_ENTRY_INHERIT_ND(cfsm::util_hfsm, entry_wait_for_signal);
  HFSM_ENTRY_INHERIT_ND(cfsm::util_hfsm, entry_transport_to_nest);
  HFSM_EXIT_INHERIT(cfsm::util_hfsm, exit_transport_to_nest);
  HFSM_ENTRY_INHERIT_ND(cfsm::util_hfsm, entry_leaving_nest);
  HFSM_STATE_DECLARE(free_block_to_nest_fsm, start, rpfsm::event_data);
  HFSM_STATE_DECLARE_ND(free_block_to_nest_fsm, acquire_block);
  HFSM_STATE_DECLARE(free_block_to_nest_fsm,
                     wait_for_pickup,
                     rpfsm::event_data);

  /* depth0 foraging states */
  HFSM_STATE_DECLARE(free_block_to_nest_fsm,
                     wait_for_drop,
                     rpfsm::event_data);

  HFSM_STATE_DECLARE_ND(free_block_to_nest_fsm, finished);

  /**
   * \brief Defines the state map for the FSM.
   *
   * Note that the order of the states in the map MUST match the order of the
   * states in \enum fsm_states, or things will not work correctly.
   */
  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
  return &mc_state_map[index];
  }

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ekST_MAX_STATES);

  /* clang-format off */
  acquire_free_block_fsm m_block_fsm;
  /* clang-format on */
};

NS_END(depth0, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH0_FREE_BLOCK_TO_NEST_FSM_HPP_ */
