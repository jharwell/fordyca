/**
 * \file crw_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH0_CRW_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH0_CRW_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "cosm/fsm/util_hfsm.hpp"
#include "cosm/fsm/explore_for_goal_fsm.hpp"
#include "cosm/fsm/metrics/goal_acq_metrics.hpp"
#include "cosm/fsm/metrics/collision_metrics.hpp"
#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/fordyca.hpp"
#include "fordyca/fsm/foraging_acq_goal.hpp"
#include "fordyca/fsm/foraging_transport_goal.hpp"
#include "cosm/robots/footbot/footbot_subsystem_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
namespace expstrat {
class foraging_expstrat;
} /* namespace expstrat */

NS_START(depth0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class crw_fsm
 * \ingroup fsm depth0
 *
 * \brief The FSM for the most basic foraging definition: each robot executing
 * this FSM roams around randomly until it finds a block, and then brings the
 * block back to the nest, and drops it.
 */
class crw_fsm final : public cfsm::util_hfsm,
                      public rer::client<crw_fsm>,
                      public cfsm::metrics::goal_acq_metrics,
                      public block_transporter,
                      public cta::taskable {
 public:
  crw_fsm(crfootbot::footbot_saa_subsystem2D* saa,
          std::unique_ptr<expstrat::foraging_expstrat> exp_behavior,
          rmath::rng* rng);

  crw_fsm(const crw_fsm&) = delete;
  crw_fsm& operator=(const crw_fsm&) = delete;

  /* collision metrics */
  bool in_collision_avoidance(void) const override RCSW_PURE;
  bool entered_collision_avoidance(void) const override RCSW_PURE;
  bool exited_collision_avoidance(void) const override RCSW_PURE;
  rtypes::timestep collision_avoidance_duration(void) const override RCSW_PURE;
  rmath::vector2u avoidance_loc(void) const override;
  /* goal acquisition metrics */
  cfsm::metrics::goal_acq_metrics::goal_type acquisition_goal(void) const override RCSW_PURE;
  exp_status is_exploring_for_goal(void) const override RCSW_PURE;
  bool is_vectoring_to_goal(void) const override { return false; }
  bool goal_acquired(void) const override RCSW_PURE;
  rmath::vector2u acquisition_loc(void) const override;
  rtypes::type_uuid entity_acquired_id(void) const override RCSW_PURE;
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector2u, current_explore_loc, const);
  RCPPSW_WRAP_OVERRIDE_DECL(rmath::vector2u, current_vector_loc, const);

  /* block transportation */
  foraging_transport_goal block_transport_goal(void) const override RCSW_PURE;

  /* taskable overrides */
  void task_execute(void) override { run(); }
  void task_start(const cta::taskable_argument*) override {}
  bool task_finished(void) const override { return m_task_finished; }
  bool task_running(void) const override { return !m_task_finished; }
  void task_reset(void) override { init(); }

  /**
   * \brief (Re)-initialize the FSM.
   */
  void init(void) override;

  /**
   * \brief Run the FSM in its current state, without injecting an event.
   */
  void run(void);

 private:
  bool block_detected(void) const;

  enum fsm_states {
    ekST_START, /* Initial state */
    ekST_ACQUIRE_BLOCK,
    ekST_TRANSPORT_TO_NEST,        /* Block found--bring it back to the nest */
    ekST_LEAVING_NEST,          /* Block dropped in nest--time to go */
    ekST_WAIT_FOR_BLOCK_PICKUP,
    ekST_WAIT_FOR_BLOCK_DROP,
    ekST_MAX_STATES
  };

  /* inherited states */
  HFSM_STATE_INHERIT(cfsm::util_hfsm, transport_to_nest,
                     rpfsm::event_data);
  HFSM_STATE_INHERIT(cfsm::util_hfsm, leaving_nest,
                     rpfsm::event_data);

  HFSM_ENTRY_INHERIT_ND(cfsm::util_hfsm, entry_transport_to_nest);
  HFSM_ENTRY_INHERIT_ND(cfsm::util_hfsm, entry_leaving_nest);
  HFSM_ENTRY_INHERIT_ND(cfsm::util_hfsm, entry_wait_for_signal);

  HFSM_EXIT_INHERIT(cfsm::util_hfsm, exit_transport_to_nest);

  /* crw fsm states */
  HFSM_STATE_DECLARE(crw_fsm, start, rpfsm::event_data);
  HFSM_STATE_DECLARE_ND(crw_fsm, acquire_block);
  HFSM_STATE_DECLARE(crw_fsm, wait_for_block_pickup,
                     rpfsm::event_data);
  HFSM_STATE_DECLARE(crw_fsm, wait_for_block_drop,
                     rpfsm::event_data);

  /**
   * \brief Defines the state map for the FSM.
   *
   * Note that the order of the states in the map MUST match the order of the
   * states in \enum fsm_states, or things will not work correctly.
   */
  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
  return (&mc_state_map[index]);
  }

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ekST_MAX_STATES);

  /* clang-format off */
  bool                       m_task_finished{false};
  cfsm::explore_for_goal_fsm m_explore_fsm;
  /* clang-format on */
};

NS_END(depth0, controller, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH0_CRW_FSM_HPP_ */
