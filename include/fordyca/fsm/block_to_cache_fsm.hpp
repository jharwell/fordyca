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

  /* taskable overrides */

  void task_execute(void) override;
  void task_start(const task_allocation::taskable_argument * arg) override;
  bool task_finished(void) const override { return ST_FINISHED == current_state(); }
  bool task_running(void) const override {
    return !(ST_FINISHED == current_state() || ST_START == current_state());
  }

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
  bool is_searching_for_block(void) const;

  /**
   * @brief Get if the robot is currently searching for a cache within the arena
   * (either vectoring towards a known cache, or exploring for one).
   *
   * @return \c TRUE if the condition is met, \c FALSE otherwise.
   */
  bool is_searching_for_cache(void) const;

  /**
   * @brief If \c TRUE, the robot is currently exploring for a block (i.e. it does
   * not know of any blocks in the arena).
   */
  bool is_exploring(void) const;

  /**
   * @brief If \c TRUE, the robot is currently vectoring towards a known block.
   */
  bool is_vectoring(void) const;

  /**
   * @brief If \c TRUE, the robot is currently engaged in collision avoidance.
   */
  bool is_avoiding_collision(void) const;

  /**
   * @brief If \c TRUE, the robot has obtained a block and is returning to the
   * nest with it.
   */
  bool is_transporting_to_nest(void) const;

  bool is_transporting_to_cache(void) const;

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
  HFSM_STATE_DECLARE(block_to_cache_fsm, start, state_machine::event_data);
  HFSM_STATE_DECLARE(block_to_cache_fsm, acquire_free_block,
                     state_machine::event_data);
  HFSM_STATE_DECLARE(block_to_cache_fsm, transport_to_cache,
                     state_machine::event_data);
  HFSM_STATE_DECLARE_ND(block_to_cache_fsm, finished);

  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
  return &mc_state_map[index];
  }

  block_to_cache_fsm(const block_to_cache_fsm& fsm) = delete;
  block_to_cache_fsm& operator=(const block_to_cache_fsm& fsm) = delete;

  /* data members */
  acquire_block_fsm m_block_fsm;
  acquire_cache_fsm m_cache_fsm;
  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_BLOCK_TO_CACHE_FSM_HPP_ */
