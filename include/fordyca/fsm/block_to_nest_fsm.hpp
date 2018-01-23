/**
 * @file block_to_nest_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_BLOCK_TO_NEST_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_BLOCK_TO_NEST_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/acquire_block_fsm.hpp"
#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/depth1/acquire_cache_fsm.hpp"
#include "fordyca/fsm/vector_fsm.hpp"
#include "fordyca/metrics/collectible_metrics/fsm/depth1_metrics.hpp"
#include "fordyca/metrics/collectible_metrics/fsm/stateful_metrics.hpp"
#include "fordyca/metrics/collectible_metrics/fsm/stateless_metrics.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "rcppsw/task_allocation/taskable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params {
struct fsm_params;
}
namespace controller {
namespace depth1 {
class foraging_sensors;
}
class actuator_manager;
}
namespace representation {
class perceived_arena_map;
class block;
}

namespace task_allocation = rcppsw::task_allocation;
namespace visitor = rcppsw::patterns::visitor;

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_to_nest_fsm
 * @ingroup fsm
 *
 * @brief Each robot executing this FSM will locate for a block (either a known
 * block or via random exploration), pickup the block and bring it all the way
 * back to the nest.
 *
 * It can be directed to acquire a block either from a cache or to find a free
 * one.
 */
class block_to_nest_fsm
    : public base_foraging_fsm,
      public metrics::collectible_metrics::fsm::stateless_metrics,
      public metrics::collectible_metrics::fsm::stateful_metrics,
      public metrics::collectible_metrics::fsm::depth1_metrics,
      public task_allocation::taskable,
      public visitor::visitable_any<block_to_nest_fsm> {
 public:
  block_to_nest_fsm(
      const struct params::fsm_params* params,
      const std::shared_ptr<rcppsw::er::server>& server,
      const std::shared_ptr<controller::depth1::foraging_sensors>& sensors,
      const std::shared_ptr<controller::actuator_manager>& actuators,
      const std::shared_ptr<representation::perceived_arena_map>& map);

  block_to_nest_fsm(const block_to_nest_fsm& fsm) = delete;
  block_to_nest_fsm& operator=(const block_to_nest_fsm& fsm) = delete;

  /* taskable overrides */
  void task_execute(void) override;
  void task_start(const task_allocation::taskable_argument* arg) override;
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

  /* base metrics */
  bool is_exploring_for_block(void) const override;
  bool is_avoiding_collision(void) const override;
  bool is_transporting_to_nest(void) const override;

  /* depth0 metrics */
  bool is_acquiring_block(void) const override;
  bool is_vectoring_to_block(void) const override;

  /* depth1 metrics */
  bool is_exploring_for_cache(void) const override;
  bool is_vectoring_to_cache(void) const override;
  bool is_acquiring_cache(void) const override;
  bool is_transporting_to_cache(void) const override { return false; }

  /**
   * @brief If \c TRUE, the robot has acquired (i.e. is sitting on top of) a
   * cache, and is waiting for the simulation to send it the block pickup
   * signal.
   */
  bool cache_acquired(void) const;

  /* @brief If \c TRUE, the robot has acquired (i.e. is sitting on top of) a
   * block, and is waiting for the simulation to send it the block pickup
   * signal.
   */
  bool block_acquired(void) const;

  /**
   * @brief Reset the FSM
   */
  void init(void) override;

 protected:
  enum fsm_states {
    ST_START,
    /**
     * Superstate for finding a free block.
     */
    ST_ACQUIRE_FREE_BLOCK,

    /**
     * @brief State robots wait in after acquiring a block for the simulation to
     * send them the block pickup signal. Having this extra state solves a lot
     * of handshaking/off by one issues regarding the timing of doing so.
     */
    ST_WAIT_FOR_BLOCK_PICKUP,

    /**
     * Superstate for finding a cached block.
     */
    ST_ACQUIRE_CACHED_BLOCK,

    /**
     * @brief State robots wait in after acquiring a cache for the simulation to
     * send them the block pickup signal. Having this extra state solves a lot
     * of handshaking/off by one issues regarding the timing of doing so.
     */
    ST_WAIT_FOR_CACHE_PICKUP,

    /**
     * Block found--bring it back to the nest.
     */
    ST_TRANSPORT_TO_NEST,

    /**
     * Obstacle nearby--avoid it.
     */
    ST_COLLISION_AVOIDANCE,

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
  HFSM_STATE_INHERIT_ND(base_foraging_fsm, collision_avoidance);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_transport_to_nest);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_collision_avoidance);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_wait_for_signal);

  /* memory foraging states */
  HFSM_STATE_DECLARE(block_to_nest_fsm, start, state_machine::event_data);
  HFSM_STATE_DECLARE_ND(block_to_nest_fsm, acquire_free_block);
  HFSM_STATE_DECLARE(block_to_nest_fsm,
                     wait_for_block_pickup,
                     state_machine::event_data);
  HFSM_STATE_DECLARE_ND(block_to_nest_fsm, acquire_cached_block);
  HFSM_STATE_DECLARE(block_to_nest_fsm,
                     wait_for_cache_pickup,
                     state_machine::event_data);
  HFSM_STATE_DECLARE_ND(block_to_nest_fsm, finished);

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
  uint                                                  m_pickup_count{0};
  std::shared_ptr<controller::depth1::foraging_sensors> m_sensors;
  acquire_block_fsm                                     m_block_fsm;
  depth1::acquire_cache_fsm                             m_cache_fsm;
  // clang-format on
  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_BLOCK_TO_NEST_FSM_HPP_ */
