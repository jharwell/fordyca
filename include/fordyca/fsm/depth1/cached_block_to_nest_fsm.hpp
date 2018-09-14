/**
 * @file cached_block_to_nest_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH1_CACHED_BLOCK_TO_NEST_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH1_CACHED_BLOCK_TO_NEST_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/acquire_block_fsm.hpp"
#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/depth1/acquire_existing_cache_fsm.hpp"
#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "rcppsw/task_allocation/taskable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace controller {
namespace depth1 {
class sensing_subsystem;
}
class actuation_subsystem;
} // namespace controller
namespace representation {
class block;
} // namespace representation

namespace ds {
class perceived_arena_map;
} // namespace ds

namespace task_allocation = rcppsw::task_allocation;
namespace visitor = rcppsw::patterns::visitor;

NS_START(fsm, depth1);

using transport_goal_type = block_transporter::goal_type;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cached_block_to_nest_fsm
 * @ingroup fsm
 *
 * @brief Each robot executing this FSM will locate for a block (either a known
 * block or via random exploration), pickup the block and bring it all the way
 * back to the nest.
 *
 * It can be directed to acquire a block either from a cache or to find a free
 * one.
 */
class cached_block_to_nest_fsm : public base_foraging_fsm,
                                 er::client<cached_block_to_nest_fsm>,
                                 public metrics::fsm::goal_acquisition_metrics,
                                 public block_transporter,
                                 public task_allocation::taskable,
                                 public visitor::visitable_any<cached_block_to_nest_fsm> {
 public:
  cached_block_to_nest_fsm(const controller::cache_selection_matrix* sel_matrix,
                           controller::saa_subsystem* saa,
                           ds::perceived_arena_map* map);

  cached_block_to_nest_fsm(const cached_block_to_nest_fsm& fsm) = delete;
  cached_block_to_nest_fsm& operator=(const cached_block_to_nest_fsm& fsm) = delete;

  /* taskable overrides */
  void task_execute(void) override;
  bool task_finished(void) const override {
    return ST_FINISHED == current_state();
  }

  bool task_running(void) const override {
    return !(ST_FINISHED == current_state() || ST_START == current_state());
  }

  /**
   * @brief Reset the task FSM to a state where it can be started again.
   */
  void task_reset(void) override { init(); }
  void task_start(const task_allocation::taskable_argument*) override {}

  /* collision metrics */
  bool in_collision_avoidance(void) const override;
  bool entered_collision_avoidance(void) const override;
  bool exited_collision_avoidance(void) const override;
  uint collision_avoidance_duration(void) const override;

  /* goal acquisition metrics */
  bool goal_acquired(void) const override;
  FSM_WRAPPER_DECLARE(bool, is_exploring_for_goal);
  FSM_WRAPPER_DECLARE(bool, is_vectoring_to_goal);
  acquisition_goal_type acquisition_goal(void) const override;

  /* block transportation */
  FSM_WRAPPER_DECLARE(transport_goal_type, block_transport_goal);

  /**
   * @brief Reset the FSM
   */
  void init(void) override;

 protected:
  enum fsm_states {
    ST_START,
    /**
     * Superstate for finding a cached block.
     */
    ST_ACQUIRE_BLOCK,

    /**
     * @brief State robots wait in after acquiring a cache for the simulation to
     * send them the block pickup signal. Having this extra state solves a lot
     * of handshaking/off by one issues regarding the timing of doing so.
     */
    ST_WAIT_FOR_PICKUP,

    ST_WAIT_FOR_DROP,

    /**
     * Block found--bring it back to the nest.
     */
    ST_TRANSPORT_TO_NEST,

    ST_LEAVING_NEST,

    /**
     * Block has been brought to the nest successfully.
     */
    ST_FINISHED,
    ST_MAX_STATES,
  };

 private:
  /**
   * @brief It is possible that robots can be waiting indefinitely for a block
   * pickup signal that will never come once a block has been acquired if they
   * "detect" a block by sprawling across multiple blocks (i.e. all ground
   * sensors did not detect the same block).
   *
   * In that case, this timeout will cause the robot to try again to acquire a
   * block, and because of the decaying relevance of cells, it will eventually
   * pick a different block than the one that got it into this predicament, and
   * the system will be able to continue profitably.
   */
  constexpr static uint kPICKUP_TIMEOUT = 100;

  /* inherited states */
  HFSM_STATE_INHERIT(base_foraging_fsm,
                     transport_to_nest,
                     state_machine::event_data);
  HFSM_STATE_INHERIT(base_foraging_fsm,
                     leaving_nest,
                     state_machine::event_data);

  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_transport_to_nest);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_leaving_nest);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_wait_for_signal);

  /* foraging states */
  HFSM_STATE_DECLARE(cached_block_to_nest_fsm, start, state_machine::event_data);
  HFSM_STATE_DECLARE_ND(cached_block_to_nest_fsm, acquire_block);
  HFSM_STATE_DECLARE(cached_block_to_nest_fsm,
                     wait_for_pickup,
                     state_machine::event_data);
  HFSM_STATE_DECLARE(cached_block_to_nest_fsm,
                     wait_for_drop,
                     state_machine::event_data);
  HFSM_STATE_DECLARE_ND(cached_block_to_nest_fsm, finished);

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
  depth1::acquire_existing_cache_fsm m_cache_fsm;
  // clang-format on
  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(depth1, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH1_CACHED_BLOCK_TO_NEST_FSM_HPP_ */
