/**
 * @file block_to_cache_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH1_BLOCK_TO_CACHE_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH1_BLOCK_TO_CACHE_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/task_allocation/taskable.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/fsm/vector_fsm.hpp"
#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/acquire_block_fsm.hpp"
#include "fordyca/fsm/depth1/acquire_cache_fsm.hpp"
#include "fordyca/metrics/collectible_metrics/robot_metrics/stateless_metrics.hpp"
#include "fordyca/metrics/collectible_metrics/robot_metrics/stateful_metrics.hpp"
#include "fordyca/metrics/collectible_metrics/robot_metrics/depth1_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace task_allocation = rcppsw::task_allocation;
namespace visitor = rcppsw::patterns::visitor;
namespace params { struct fsm_params; }
namespace representation { class perceived_arena_map; class block; }
namespace controller {
namespace depth1{ class foraging_sensors; };
class actuator_manager;
}

NS_START(fsm, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief The FSM for the block-to-cache subtask.
 *
 * Each robot executing this FSM will locate a free block (either a known block
 * or via random exploration), pickup the block and bring it to the best cache
 * it knows about. Once it has done that it will signal that its task is
 * complete.
 */
class block_to_cache_fsm : public base_foraging_fsm,
                           public metrics::collectible_metrics::robot_metrics::stateless_metrics,
                           public metrics::collectible_metrics::robot_metrics::stateful_metrics,
                           public metrics::collectible_metrics::robot_metrics::depth1_metrics,
                           public task_allocation::taskable,
                           public visitor::visitable_any<block_to_cache_fsm> {
 public:
  block_to_cache_fsm(
      const struct params::fsm_params* params,
      const std::shared_ptr<rcppsw::er::server>& server,
      const std::shared_ptr<controller::depth1::foraging_sensors>& sensors,
      const std::shared_ptr<controller::actuator_manager>& actuators,
      const std::shared_ptr<const representation::perceived_arena_map>& map);

  /* taskable overrides */

  void task_execute(void) override;
  void task_start(const task_allocation::taskable_argument * arg) override;
  bool task_finished(void) const override { return ST_FINISHED == current_state(); }
  bool task_running(void) const override {
    return !(ST_FINISHED == current_state() || ST_START == current_state());
  }
  void task_reset(void) override { init(); }

  /* base metrics */
  bool is_exploring_for_block(void) const override;
  bool is_avoiding_collision(void) const override;
  bool is_transporting_to_nest(void) const override { return false; }

  /* depth0 metrics */
  bool is_acquiring_block(void) const override;
  bool is_vectoring_to_block(void) const override;

  /* depth1 metrics */
  bool is_exploring_for_cache(void) const override;
  bool is_vectoring_to_cache(void) const override;
  bool is_acquiring_cache(void) const override;
  bool is_transporting_to_cache(void) const override;

  bool cache_acquired(void) const;
  bool block_acquired(void) const;

  /**
   * @brief Reset the FSM
   */
  void init(void) override;

  controller::depth1::foraging_sensors* sensors(void) const { return m_sensors.get(); }

 protected:
  enum fsm_states {
    ST_START,
    ST_ACQUIRE_FREE_BLOCK,    /* superstate for finding a free block */
    ST_WAIT_FOR_BLOCK_PICKUP,
    ST_TRANSPORT_TO_CACHE,    /* Block found--bring it back to a cache */
    ST_WAIT_FOR_CACHE_DROP,
    ST_COLLISION_AVOIDANCE,
    ST_FINISHED,
    ST_MAX_STATES,
  };

 private:
  constexpr static uint kPICKUP_TIMEOUT = 100;

  /* inherited states */
  HFSM_STATE_INHERIT_ND(base_foraging_fsm, collision_avoidance);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_collision_avoidance);

  /* block to cache states */
  HFSM_STATE_DECLARE(block_to_cache_fsm, start, state_machine::event_data);
  HFSM_STATE_DECLARE_ND(block_to_cache_fsm, acquire_free_block);
  HFSM_STATE_DECLARE(block_to_cache_fsm, wait_for_block_pickup,
                     state_machine::event_data);
  HFSM_STATE_DECLARE_ND(block_to_cache_fsm, transport_to_cache);
  HFSM_STATE_DECLARE(block_to_cache_fsm, wait_for_cache_drop,
                     state_machine::event_data);
  HFSM_STATE_DECLARE_ND(block_to_cache_fsm, finished);
  HFSM_ENTRY_DECLARE_ND(block_to_cache_fsm, entry_wait_for_pickup);

  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
  return &mc_state_map[index];
  }

  block_to_cache_fsm(const block_to_cache_fsm& fsm) = delete;
  block_to_cache_fsm& operator=(const block_to_cache_fsm& fsm) = delete;

  uint                                                  m_pickup_count;
  std::shared_ptr<controller::depth1::foraging_sensors> m_sensors;
  acquire_block_fsm                                     m_block_fsm;
  acquire_cache_fsm                                     m_cache_fsm;
  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(depth1, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH1_BLOCK_TO_CACHE_FSM_HPP_ */
