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

#ifndef INCLUDE_FORDYCA_FSM_BLOCK_TO_CACHE_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_BLOCK_TO_CACHE_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <utility>

#include "rcppsw/task_allocation/taskable.hpp"
#include "fordyca/fsm/vector_fsm.hpp"
#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/acquire_block_fsm.hpp"
#include "fordyca/fsm/acquire_cache_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params {
struct fsm_params;
} /* namespace params */

namespace controller {
class sensor_manager;
class actuator_manager;
} /* namespace controller */

namespace representation {
class perceived_arena_map;
class block;
} /* namespace representation */

namespace task_allocation = rcppsw::task_allocation;

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @brief The FSM for the block-to-cache subtask.
 *
 * Each robot executing this FSM will locate for a block (either a known block or
 * via random exploration), pickup the block and bring it to the best cache it
 * knows about.
 */
class block_to_cache_fsm : public base_foraging_fsm,
                           public task_allocation::taskable {
 public:
  block_to_cache_fsm(
      const struct params::fsm_params* params,
      const std::shared_ptr<rcppsw::common::er_server>& server,
      const std::shared_ptr<controller::sensor_manager>& sensors,
      const std::shared_ptr<controller::actuator_manager>& actuators,
      const std::shared_ptr<const representation::perceived_arena_map>& map);

  /**
   * @brief Reset the FSM
   */
  void init(void) override;

  /**
   * @brief Get if the robot is currently searching for a block within the arena
   * (either vectoring towards a known block, or exploring for one).
   *
   * @return TRUE if the condition is met, FALSE otherwise.
   */
  bool is_searching_for_block(void) const {
    return m_block_fsm.is_searching_for_block();
  }

  /**
   * @brief Get if the robot is currently searching for a cache within the arena
   * (either vectoring towards a known cache, or exploring for one).
   *
   * @return TRUE if the condition is met, FALSE otherwise.
   */
  bool is_searching_for_cache(void) const {
    return m_cache_fsm.is_searching_for_cache();
  }

  bool is_exploring(void) const {
    return m_block_fsm.is_exploring() || m_cache_fsm.is_exploring();
  }
  bool is_vectoring(void) const {
    return m_block_fsm.is_vectoring() || m_cache_fsm.is_vectoring();
  }

  bool is_avoiding_collision(void) const {
    return m_block_fsm.is_avoiding_collision() ||
        m_cache_fsm.is_avoiding_collision();
  }
  bool is_transporting_to_cache(void) const {
    return current_state() == ST_TRANSPORT_TO_CACHE;
  }

  /**
   * @brief Run the FSM in its current state without injecting an event into it.
   */
  void task_execute(void) override;
  void task_start(const task_allocation::taskable_argument * arg) override;
  bool task_finished(void) const override { return ST_FINISHED == current_state(); }

 protected:
  enum fsm_states {
    ST_START,
    ST_ACQUIRE_FREE_BLOCK,    /* superstate for finding a free block */
    ST_TRANSPORT_TO_CACHE,    /* Block found--bring it back to a cache */
    ST_FINISHED,
    ST_MAX_STATES,
  };

 private:
  /* memory foraging states */
  HFSM_STATE_DECLARE(block_to_cache_fsm, start, state_machine::no_event_data);
  HFSM_STATE_DECLARE(block_to_cache_fsm, acquire_free_block,
                     state_machine::event_data);
  HFSM_STATE_DECLARE(block_to_cache_fsm, transport_to_cache,
                     state_machine::event_data);
  HFSM_STATE_DECLARE(block_to_cache_fsm, finished,
                     state_machine::no_event_data);

  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
  HFSM_DEFINE_STATE_MAP(state_map_ex, kSTATE_MAP) {
    HFSM_STATE_MAP_ENTRY_EX(&start, hfsm::top_state()),
        HFSM_STATE_MAP_ENTRY_EX(&acquire_free_block, hfsm::top_state()),
        HFSM_STATE_MAP_ENTRY_EX(&transport_to_cache, hfsm::top_state()),
        HFSM_STATE_MAP_ENTRY_EX(&finished, hfsm::top_state()),
        };
  HFSM_VERIFY_STATE_MAP(state_map_ex, kSTATE_MAP);
  return &kSTATE_MAP[index];
  }

  block_to_cache_fsm(const block_to_cache_fsm& fsm) = delete;
  block_to_cache_fsm& operator=(const block_to_cache_fsm& fsm) = delete;

  /* data members */
  acquire_block_fsm m_block_fsm;
  acquire_cache_fsm m_cache_fsm;
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_BLOCK_TO_CACHE_FSM_HPP_ */