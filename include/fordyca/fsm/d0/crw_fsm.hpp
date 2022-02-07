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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "cosm/foraging/fsm/foraging_util_hfsm.hpp"
#include "cosm/spatial/fsm/explore_for_goal_fsm.hpp"
#include "cosm/spatial/metrics/goal_acq_metrics.hpp"
#include "cosm/spatial/metrics/interference_metrics.hpp"
#include "cosm/fsm/block_transporter.hpp"
#include "cosm/subsystem/subsystem_fwd.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics.hpp"

#include "fordyca/fsm/foraging_transport_goal.hpp"
#include "fordyca/fordyca.hpp"
#include "fordyca/fsm/foraging_acq_goal.hpp"
#include "fordyca/fsm/fsm_ro_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, d0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class crw_fsm
 * \ingroup fsm d0
 *
 * \brief The FSM for the most basic foraging definition: each robot executing
 * this FSM roams around randomly until it finds a block, and then brings the
 * block back to the nest, and drops it.
 */
class crw_fsm final : public cffsm::foraging_util_hfsm,
                      public rer::client<crw_fsm>,
                      public csmetrics::goal_acq_metrics,
                      public cfsm::block_transporter<foraging_transport_goal>,
                      public cfsm::metrics::block_transporter_metrics,
                      public cta::taskable {
 public:
  crw_fsm(const csfsm::fsm_params* params,
          std::unique_ptr<csstrategy::base_strategy> explore,
          std::unique_ptr<cssnest_acq::base_nest_acq> nest_acq,
          const rmath::vector2d& nest_loc,
          rmath::rng* rng);

  crw_fsm(const crw_fsm&) = delete;
  crw_fsm& operator=(const crw_fsm&) = delete;

  /* goal acquisition metrics */
  csmetrics::goal_acq_metrics::goal_type acquisition_goal(void) const override RCPPSW_PURE;
  exp_status is_exploring_for_goal(void) const override RCPPSW_PURE;
  bool is_vectoring_to_goal(void) const override { return false; }
  bool goal_acquired(void) const override RCPPSW_PURE;
  rmath::vector3z acquisition_loc3D(void) const override RCPPSW_PURE;
  rtypes::type_uuid entity_acquired_id(void) const override RCPPSW_PURE;
  RCPPSW_WRAP_DECL_OVERRIDE(rmath::vector3z, explore_loc3D, const);
  RCPPSW_WRAP_DECL_OVERRIDE(rmath::vector3z, vector_loc3D, const);

  /* block transportation */
  foraging_transport_goal block_transport_goal(void) const override RCPPSW_PURE;
  bool is_phototaxiing_to_goal(bool include_ca) const override RCPPSW_PURE;

  /* taskable overrides */
  void task_execute(void) override { run(); }
  void task_start(cta::taskable_argument*) override {}
  bool task_finished(void) const override { return m_task_finished; }
  bool task_running(void) const override { return !m_task_finished; }
  void task_reset(void) override;

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
    ekST_TRANSPORT_TO_NEST,     /* Block found--bring it back to the nest */
    ekST_LEAVING_NEST,          /* Block dropped in nest--time to go */
    ekST_WAIT_FOR_BLOCK_PICKUP,
    ekST_WAIT_FOR_BLOCK_DROP,
    ekST_MAX_STATES
  };

  /* inherited states */
  RCPPSW_HFSM_STATE_INHERIT(cffsm::foraging_util_hfsm,
                     transport_to_nest,
                     nest_transport_data);
  RCPPSW_HFSM_STATE_INHERIT(cffsm::foraging_util_hfsm, leaving_nest,
                     rpfsm::event_data);

  RCPPSW_HFSM_ENTRY_INHERIT_ND(cffsm::foraging_util_hfsm,
                               entry_transport_to_nest);
  RCPPSW_HFSM_ENTRY_INHERIT_ND(cffsm::foraging_util_hfsm, entry_leaving_nest);
  RCPPSW_HFSM_ENTRY_INHERIT_ND(csfsm::util_hfsm, entry_wait_for_signal);

  RCPPSW_HFSM_EXIT_INHERIT(cffsm::foraging_util_hfsm, exit_transport_to_nest);

  /* crw fsm states */
  RCPPSW_HFSM_STATE_DECLARE(crw_fsm, start, rpfsm::event_data);
  RCPPSW_HFSM_STATE_DECLARE_ND(crw_fsm, acquire_block);
  RCPPSW_HFSM_STATE_DECLARE(crw_fsm, wait_for_block_pickup,
                     rpfsm::event_data);
  RCPPSW_HFSM_STATE_DECLARE(crw_fsm, wait_for_block_drop,
                     rpfsm::event_data);

  /**
   * \brief Defines the state map for the FSM.
   *
   * Note that the order of the states in the map MUST match the order of the
   * states in \enum fsm_states, or things will not work correctly.
   */
  RCPPSW_HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
  return (&mc_state_map[index]);
  }

  RCPPSW_HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ekST_MAX_STATES);

  /* clang-format off */
  const rmath::vector2d       mc_nest_loc;

  bool                        m_task_finished{false};
  size_t                      m_pickup_wait_count{0};
  csfsm::explore_for_goal_fsm m_explore_fsm;
  /* clang-format on */
};

NS_END(d0, controller, fordyca);
