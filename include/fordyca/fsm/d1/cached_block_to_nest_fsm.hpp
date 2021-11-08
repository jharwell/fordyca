/**
 * \file cached_block_to_nest_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_D1_CACHED_BLOCK_TO_NEST_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_D1_CACHED_BLOCK_TO_NEST_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include "cosm/foraging/fsm/foraging_util_hfsm.hpp"
#include "cosm/ta/taskable.hpp"
#include "cosm/fsm/block_transporter.hpp"
#include "cosm/spatial/metrics/goal_acq_metrics.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics.hpp"

#include "fordyca/fsm/acquire_existing_cache_fsm.hpp"
#include "fordyca/fsm/fsm_ro_params.hpp"
#include "fordyca/fsm/foraging_transport_goal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace repr {
class block;
} // namespace repr

namespace ds {
class dpo_store;
} // namespace ds

NS_START(fsm, d1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cached_block_to_nest_fsm
 * \ingroup fsm
 *
 * \brief Each robot executing this FSM will locate for a block (either a known
 * block or via exploration), pickup the block and bring it all the way back to
 * the nest.
 *
 * It can be directed to acquire a block either from a cache or to find a free
 * one.
 */
class cached_block_to_nest_fsm final : public cffsm::foraging_util_hfsm,
                                       public rer::client<cached_block_to_nest_fsm>,
                                       public csmetrics::goal_acq_metrics,
                                       public cfsm::block_transporter<foraging_transport_goal>,
                                       public cfsm::metrics::block_transporter_metrics,
                                       public cta::taskable {
 public:
  cached_block_to_nest_fsm(
      const fsm_ro_params* c_ro,
      const csfsm::fsm_params* c_no,
      std::unique_ptr<csstrategy::base_strategy> explore,
      std::unique_ptr<cssnest_acq::base_nest_acq> nest_acq,
      rmath::rng *rng);
  ~cached_block_to_nest_fsm(void) override = default;

  cached_block_to_nest_fsm(const cached_block_to_nest_fsm&) = delete;
  cached_block_to_nest_fsm& operator=(const cached_block_to_nest_fsm&) = delete;

  /* taskable overrides */
  void task_execute(void) override;
  bool task_finished(void) const override {
    return ekST_FINISHED == current_state();
  }

  bool task_running(void) const override {
    return !(ekST_FINISHED == current_state() || ekST_START == current_state());
  }

  /**
   * \brief Reset the task FSM to a state where it can be started again.
   */
  void task_reset(void) override { init(); }
  void task_start(cta::taskable_argument*) override {}

  /* collision metrics */
  bool exp_interference(void) const override RCPPSW_PURE;
  bool entered_interference(void) const override RCPPSW_PURE;
  bool exited_interference(void) const override RCPPSW_PURE;
  rtypes::timestep interference_duration(void) const override RCPPSW_PURE;
  RCPPSW_WRAP_DECL_OVERRIDE(rmath::vector3z, interference_loc3D, const);

  /* goal acquisition metrics */
  bool goal_acquired(void) const override RCPPSW_PURE;
  RCPPSW_WRAP_DECL_OVERRIDE(exp_status, is_exploring_for_goal, const);
  RCPPSW_WRAP_DECL_OVERRIDE(bool, is_vectoring_to_goal, const);
  RCPPSW_WRAP_DECL_OVERRIDE(rmath::vector3z, acquisition_loc3D, const);
  RCPPSW_WRAP_DECL_OVERRIDE(rmath::vector3z, explore_loc3D, const);
  RCPPSW_WRAP_DECL_OVERRIDE(rmath::vector3z, vector_loc3D, const);
  RCPPSW_WRAP_DECL_OVERRIDE(rtypes::type_uuid, entity_acquired_id, const);
  csmetrics::goal_acq_metrics::goal_type acquisition_goal(void) const override RCPPSW_PURE;

  /* block transportation */
  foraging_transport_goal block_transport_goal(void) const override RCPPSW_PURE;
  bool is_phototaxiing_to_goal(bool include_ca) const override RCPPSW_PURE;

  /**
   * \brief Reset the FSM
   */
  void init(void) override;

 protected:
  enum fsm_states {
    ekST_START,
    /**
     * Superstate for finding a cached block.
     */
    ekST_ACQUIRE_BLOCK,

    /**
     * \brief State robots wait in after acquiring a cache for the simulation to
     * send them the block pickup signal. Having this extra state solves a lot
     * of handshaking/off by one issues regarding the timing of doing so.
     */
    ekST_WAIT_FOR_PICKUP,

    ekST_WAIT_FOR_DROP,

    /**
     * Block found--bring it back to the nest.
     */
    ekST_TRANSPORT_TO_NEST,

    ekST_LEAVING_NEST,

    /**
     * Block has been brought to the nest successfully.
     */
    ekST_FINISHED,
    ekST_MAX_STATES,
  };

 private:
  /**
   * \brief It is possible that robots can be waiting indefinitely for a block
   * pickup signal that will never come once a block has been acquired if they
   * "detect" a block by sprawling across multiple blocks (i.e. all ground
   * sensors did not detect the same block).
   *
   * In that case, this timeout will cause the robot to try again to acquire a
   * block, and because of the decaying relevance of cells, it will eventually
   * pick a different block than the one that got it into this predicament, and
   * the system will be able to continue profitably.
   */
  static constexpr uint kPICKUP_TIMEOUT = 100;

  /* inherited states */
  RCPPSW_HFSM_STATE_INHERIT(cffsm::foraging_util_hfsm,
                     transport_to_nest,
                     nest_transport_data);
  RCPPSW_HFSM_STATE_INHERIT(cffsm::foraging_util_hfsm,
                     leaving_nest,
                     rpfsm::event_data);

  RCPPSW_HFSM_ENTRY_INHERIT_ND(cffsm::foraging_util_hfsm, entry_transport_to_nest);
  RCPPSW_HFSM_ENTRY_INHERIT_ND(cffsm::foraging_util_hfsm, entry_leaving_nest);
  RCPPSW_HFSM_ENTRY_INHERIT_ND(csfsm::util_hfsm, entry_wait_for_signal);

  RCPPSW_HFSM_EXIT_INHERIT(cffsm::foraging_util_hfsm, exit_transport_to_nest);

  /* foraging states */
  RCPPSW_HFSM_STATE_DECLARE(cached_block_to_nest_fsm, start, rpfsm::event_data);
  RCPPSW_HFSM_STATE_DECLARE_ND(cached_block_to_nest_fsm, acquire_block);
  RCPPSW_HFSM_STATE_DECLARE(cached_block_to_nest_fsm,
                     wait_for_pickup,
                     rpfsm::event_data);
  RCPPSW_HFSM_STATE_DECLARE(cached_block_to_nest_fsm,
                     wait_for_drop,
                     rpfsm::event_data);
  RCPPSW_HFSM_STATE_DECLARE_ND(cached_block_to_nest_fsm, finished);

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
  const rmath::vector2d      mc_nest_loc;
  acquire_existing_cache_fsm m_cache_fsm;
  /* clang-format on */
};

NS_END(d1, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_D1_CACHED_BLOCK_TO_NEST_FSM_HPP_ */
