/**
 * \file block_to_goal_fsm.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"

#include "cosm/foraging/fsm/foraging_util_hfsm.hpp"
#include "cosm/fsm/block_transporter.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics.hpp"
#include "cosm/spatial/metrics/goal_acq_metrics.hpp"
#include "cosm/subsystem/subsystem_fwd.hpp"
#include "cosm/ta/taskable.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/fsm/foraging_transport_goal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::fsm {
class acquire_goal_fsm;
} /* namespace cosm::spatial::fsm */

NS_START(fordyca, fsm);

class acquire_free_block_fsm;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_to_goal_fsm
 * \ingroup fsm d1
 *
 * \brief Base FSM for acquiring, picking up a block, and then bringing it
 * somewhere and dropping it.
 *
 * Each robot executing this FSM will locate a free block (either a known block
 * or via random exploration), pickup the block and bring it to its chosen
 * goal. Once it has done that it will signal that its task is complete.
 */
class block_to_goal_fsm : public rer::client<block_to_goal_fsm>,
                          public cffsm::foraging_util_hfsm,
                          public cta::taskable,
                          public csmetrics::goal_acq_metrics,
                          public cfsm::block_transporter<foraging_transport_goal>,
                          public cfsm::metrics::block_transporter_metrics {
 public:
  block_to_goal_fsm(csfsm::acquire_goal_fsm* goal_fsm,
                    csfsm::acquire_goal_fsm* block_fsm,
                    const csfsm::fsm_params* c_no,
                    rmath::rng* rng);
  ~block_to_goal_fsm(void) override = default;

  block_to_goal_fsm(const block_to_goal_fsm&) = delete;
  block_to_goal_fsm& operator=(const block_to_goal_fsm&) = delete;

  /* taskable overrides */
  void task_execute(void) override;
  void task_start(cta::taskable_argument* arg) override;
  bool task_finished(void) const override {
    return ekST_FINISHED == current_state();
  }
  bool task_running(void) const override {
    return !(ekST_FINISHED == current_state() || ekST_START == current_state());
  }
  void task_reset(void) override { init(); }

  /* collision metrics */
  bool exp_interference(void) const override final RCPPSW_PURE;
  bool entered_interference(void) const override final RCPPSW_PURE;
  bool exited_interference(void) const override final RCPPSW_PURE;
  boost::optional<rtypes::timestep> interference_duration(void) const override final RCPPSW_PURE;
  boost::optional<rmath::vector3z> interference_loc3D(void) const override final RCPPSW_PURE;

  /* goal acquisition metrics */
  bool is_vectoring_to_goal(void) const override final RCPPSW_PURE;
  exp_status is_exploring_for_goal(void) const override final RCPPSW_PURE;
  bool goal_acquired(void) const override RCPPSW_PURE;
  csmetrics::goal_acq_metrics::goal_type acquisition_goal(void) const override;
  boost::optional<rmath::vector3z> acquisition_loc3D(void) const override final RCPPSW_PURE;
  boost::optional<rmath::vector3z> explore_loc3D(void) const override final RCPPSW_PURE;
  boost::optional<rmath::vector3z> vector_loc3D(void) const override final RCPPSW_PURE;

  /**
   * \brief Reset the FSM
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

  const csfsm::acquire_goal_fsm* goal_fsm(void) const { return m_goal_fsm; }
  const csfsm::acquire_goal_fsm* block_fsm(void) const { return m_block_fsm; }

 private:
  /* inherited states */
  RCPPSW_HFSM_ENTRY_INHERIT_ND(csfsm::util_hfsm, entry_wait_for_signal);

  /* block to goal states */
  RCPPSW_HFSM_STATE_DECLARE(block_to_goal_fsm, start, rpfsm::event_data);
  RCPPSW_HFSM_STATE_DECLARE_ND(block_to_goal_fsm, acquire_block);
  RCPPSW_HFSM_STATE_DECLARE(block_to_goal_fsm,
                            wait_for_block_pickup,
                            rpfsm::event_data);
  RCPPSW_HFSM_STATE_DECLARE_ND(block_to_goal_fsm, transport_to_goal);
  RCPPSW_HFSM_STATE_DECLARE(block_to_goal_fsm,
                            wait_for_block_drop,
                            rpfsm::event_data);
  RCPPSW_HFSM_STATE_DECLARE_ND(block_to_goal_fsm, finished);

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
  csfsm::acquire_goal_fsm* const  m_goal_fsm;
  csfsm::acquire_goal_fsm * const m_block_fsm;
  /* clang-format on */
};

NS_END(fsm, fordyca);
