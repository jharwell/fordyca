/**
 * \file dpo_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_D0_DPO_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_D0_DPO_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "cosm/foraging/fsm/foraging_util_hfsm.hpp"
#include "cosm/fsm/block_transporter.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics.hpp"

#include "fordyca/fsm/d0/free_block_to_nest_fsm.hpp"
#include "fordyca/fsm/fsm_ro_params.hpp"
#include "fordyca/fsm/foraging_transport_goal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace ds { class dpo_store; }

NS_START(fsm, d0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dpo_fsm
 * \ingroup fsm d0
 *
 * \brief The FSM for an unpartitioned foraging task. Each robot executing this
 * FSM will locate for a block (either a known block or via exploration), pickup
 * the block and bring it all the way back to the nest.
 *
 * This FSM will only pickup free blocks. Once it has brought a block all the
 * way to the nest and dropped it in the nest, it will repeat the same sequence
 * (i.e. it loops indefinitely).
 */
class dpo_fsm final : public cffsm::foraging_util_hfsm,
                      public rer::client<dpo_fsm>,
                      public csmetrics::goal_acq_metrics,
                      public cfsm::metrics::block_transporter_metrics,
                      public cfsm::block_transporter<foraging_transport_goal>,
                      public cta::taskable {
 public:
  dpo_fsm(const fsm_ro_params * params,
          crfootbot::footbot_saa_subsystem* saa,
          std::unique_ptr<csstrategy::base_strategy> explore,
          std::unique_ptr<csstrategy::base_strategy> nest_acq,
          rmath::rng* rng);
  ~dpo_fsm(void) override = default;
  dpo_fsm(const dpo_fsm&) = delete;
  dpo_fsm& operator=(const dpo_fsm&) = delete;

  /*
   * Taskable overrides. The DPO FSM is not really a task, but needs to behave
   * as one for the purpose of being able to use it with the \ref
   * supervisor_fsm.
   */
  void task_execute(void) override { run(); }
  void task_start(cta::taskable_argument*) override {}
  bool task_finished(void) const override { return m_task_finished; }
  bool task_running(void) const override { return !m_task_finished; }
  void task_reset(void) override { init(); }

  /* collision metrics */
  RCPPSW_WRAP_OVERRIDE_DECL(bool, exp_interference, const);
  RCPPSW_WRAP_OVERRIDE_DECL(bool, entered_interference, const);
  RCPPSW_WRAP_OVERRIDE_DECL(bool, exited_interference, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rtypes::timestep, interference_duration, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector3z, interference_loc3D, const);

  /* goal acquisition metrics */
  RCPPSW_WRAP_OVERRIDE_DECL(exp_status, is_exploring_for_goal, const);
  RCPPSW_WRAP_OVERRIDE_DECL(bool, is_vectoring_to_goal, const);
  RCPPSW_WRAP_OVERRIDE_DECL(bool, goal_acquired, const);
  RCPPSW_WRAP_OVERRIDE_DECL(csmetrics::goal_acq_metrics::goal_type,
                            acquisition_goal,
                            const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector3z, acquisition_loc3D, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector3z, explore_loc3D, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector3z, vector_loc3D, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rtypes::type_uuid, entity_acquired_id, const);

  /* block transportation */
  RCPPSW_WRAP_OVERRIDE_DECL(foraging_transport_goal,
                            block_transport_goal,
                            const);
  RCPPSW_WRAP_OVERRIDE_DECL(bool, is_phototaxiing_to_goal, const);

  void init(void) override;

  /**
   * \brief Run the FSM in its current state, without injecting an event.
   */
  void run(void);

 protected:
  enum fsm_states {
    ekST_START,
    ekST_BLOCK_TO_NEST,     /* Find a block and bring it to the nest */
    ekST_LEAVING_NEST,      /* Block dropped in nest--time to go */
    ekST_MAX_STATES
  };

 private:
  /* inherited states */
  RCPPSW_HFSM_STATE_INHERIT(cffsm::foraging_util_hfsm, leaving_nest,
                     rpfsm::event_data);
  RCPPSW_HFSM_ENTRY_INHERIT_ND(cffsm::foraging_util_hfsm, entry_leaving_nest);

  /* foraging states */
  RCPPSW_HFSM_STATE_DECLARE(dpo_fsm, start, rpfsm::event_data);
  RCPPSW_HFSM_STATE_DECLARE(dpo_fsm, block_to_nest, rpfsm::event_data);

  /**
   * \brief Defines the state map for the FSM.
   *
   * Note that the order of the states in the map MUST match the order of the
   * states in \enum fsm_states, or things will not work correctly.
   */
  RCPPSW_HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
  return &mc_state_map[index];
  }

  RCPPSW_HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ekST_MAX_STATES);

  /* clang-format off */
  bool                   m_task_finished{false};
  free_block_to_nest_fsm m_block_fsm;
  /* clang-format on */
};

NS_END(d0, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_D0_DPO_FSM_HPP_ */
