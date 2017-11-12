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
#include "rcppsw/task_allocation/taskable.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/fsm/depth0/vector_fsm.hpp"
#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/acquire_block_fsm.hpp"
#include "fordyca/fsm/depth1/acquire_cache_fsm.hpp"
#include "fordyca/diagnostics/depth1/collectible_diagnostics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params { struct fsm_params; }
namespace controller {
namespace depth1{ class foraging_sensors; }
class actuator_manager;
}
namespace representation { class perceived_arena_map; class block; }

namespace task_allocation = rcppsw::task_allocation;
namespace visitor = rcppsw::patterns::visitor;

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @brief The FSM for an unpartitioned foraging task.
 *
 * Each robot executing this FSM will locate for a block (either a known block
 * or via random exploration), pickup the block and bring it all the way back to
 * the nest.
 */
class block_to_nest_fsm : public base_foraging_fsm,
                          public diagnostics::depth1::collectible_diagnostics,
                          public task_allocation::taskable,
                          public visitor::visitable_any<block_to_nest_fsm> {
 public:
  block_to_nest_fsm(
      const struct params::fsm_params* params,
      const std::shared_ptr<rcppsw::common::er_server>& server,
      const std::shared_ptr<controller::depth1::foraging_sensors>& sensors,
      const std::shared_ptr<controller::actuator_manager>& actuators,
      const std::shared_ptr<const representation::perceived_arena_map>& map);

  /* taskable overrides */

  /**
   * @brief Run the next step of the FSM during task execution.
   */
  void task_execute(void) override;

  /**
   * @brief Start/restart the FSM. It can be told to retrieve a block from a
   * cache or to look for a free block, via the argument.
   */
  void task_start(const task_allocation::taskable_argument * arg) override;

  /**
   * @brief Determine if a block has been brought to the nest, and the robot has
   * subsequently left the nest, ready for its next task.
   *
   * @return \c TRUE if the condition is met, \c FALSE otherwise.
   */
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

  /* base diagnostics */
  bool is_exploring_for_block(void) const override;
  bool is_avoiding_collision(void) const override;
  bool is_transporting_to_nest(void) const override;

  /* depth0 diagnostics */
  bool is_acquiring_block(void) const override;
  bool is_vectoring_to_block(void) const override;

  /* depth1 diagnostics */
  bool is_exploring_for_cache(void) const override;
  bool is_vectoring_to_cache(void) const override;
  bool is_acquiring_cache(void) const override;
  bool is_transporting_to_cache(void) const override { return false; };

  /**
   * @brief Reset the FSM
   */
  void init(void) override;

  controller::depth1::foraging_sensors* sensors(void) const { return m_sensors.get(); }

 protected:
  enum fsm_states {
    ST_START,
    ST_ACQUIRE_FREE_BLOCK,    /* superstate for finding a  free block */
    ST_ACQUIRE_CACHED_BLOCK,  /* superstate for finding a cached block */
    ST_TRANSPORT_TO_NEST,        /* Block found--bring it back to the nest */
    ST_COLLISION_AVOIDANCE,
    ST_FINISHED,
    ST_MAX_STATES,
  };

 private:
  /* inherited states */
  HFSM_STATE_INHERIT(base_foraging_fsm, transport_to_nest,
                     state_machine::event_data);
  HFSM_STATE_INHERIT_ND(base_foraging_fsm, collision_avoidance);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_transport_to_nest);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_collision_avoidance);

  /* memory foraging states */
  HFSM_STATE_DECLARE(block_to_nest_fsm, start, state_machine::event_data);
  HFSM_STATE_DECLARE(block_to_nest_fsm, acquire_free_block,
                     state_machine::event_data);
  HFSM_STATE_DECLARE(block_to_nest_fsm, acquire_cached_block,
                     state_machine::event_data);
  HFSM_STATE_DECLARE_ND(block_to_nest_fsm, finished);

  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
  return &mc_state_map[index];
  }

  block_to_nest_fsm(const block_to_nest_fsm& fsm) = delete;
  block_to_nest_fsm& operator=(const block_to_nest_fsm& fsm) = delete;

  /* data members */
  std::shared_ptr<controller::depth1::foraging_sensors>  m_sensors;
  acquire_block_fsm m_block_fsm;
  depth1::acquire_cache_fsm m_cache_fsm;
  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_BLOCK_TO_NEST_FSM_HPP_ */
