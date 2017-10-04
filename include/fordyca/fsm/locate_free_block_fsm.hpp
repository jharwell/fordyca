/**
 * @file locate_free_block_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_LOCATE_FREE_BLOCK_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_LOCATE_FREE_BLOCK_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <utility>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>

#include "rcppsw/task_allocation/taskable.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/vector_fsm.hpp"
#include "fordyca/fsm/explore_fsm.hpp"

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

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @brief The FSM for an unpartitioned foraging task. Each robot executing this
 * FSM will locate for a block (either a known block or via random exploration),
 * pickup the block and bring it all the way back to the nest.
 */
class locate_free_block_fsm : public base_foraging_fsm,
                              public  rcppsw::task_allocation::taskable {
 public:
  locate_free_block_fsm(
      const struct params::fsm_params* params,
      const std::shared_ptr<rcppsw::common::er_server>& server,
      const std::shared_ptr<controller::sensor_manager>& sensors,
      const std::shared_ptr<controller::actuator_manager>& actuators,
      const std::shared_ptr<const representation::perceived_arena_map>& map);

  /**
   * @brief Reset the FSM
   */
  void init(void);

  /**
   * @brief Get if the robot is currently searching for a block within the arena
   * (either vectoring towards a known block, or exploring for one).
   *
   * @return TRUE if the condition is met, FALSE otherwise.
   */
  bool is_searching_for_block(void) const {
    return is_vectoring() || is_exploring();
  }

  bool is_exploring(void) const {
    return (current_state() == ST_ACQUIRE_BLOCK && m_explore_fsm.is_searching()); }

  bool is_vectoring(void) const {
    return current_state() == ST_ACQUIRE_BLOCK && m_vector_fsm.in_progress();
  }

  void task_execute(void);
  bool task_finished(void) const { return ST_FINISHED == current_state(); }

 protected:
  enum fsm_states {
    ST_START,
    ST_ACQUIRE_BLOCK, /* superstate for finding a free block */
    ST_FINISHED,
    ST_MAX_STATES
  };

 private:
  /**
   * @brief Get the previous state of the FSM. Note that this is not necessarily the state
   * that the FSM was in last time the state engine was run, but that this is
   * the last visited state that is NOT the current state.
   */
  uint8_t previous_state(void) const { return m_previous_state; }
  uint8_t current_state(void) const { return m_current_state; }
  uint8_t max_states(void) const { return ST_MAX_STATES; }
  uint8_t next_state(void) const { return m_next_state; }
  uint8_t initial_state(void) const { return m_initial_state; }
  void next_state(uint8_t next_state) { m_next_state = next_state; }
  uint8_t last_state(void) const { return m_last_state; }
  void update_state(uint8_t update_state);

  /**
   * @brief Acquire a free block.
   *
   * @return TRUE if a block has been acquired, FALSE otherwise.
   */
  bool acquire_free_block(void);

  /**
   * @brief Acquire a known block. If the robot's knowledge of the chosen
   * block's existence expires during the pursuit of a known block, that is
   * ignored.
   */
  void acquire_known_block(
      std::list<std::pair<const representation::block*, double>> blocks);

  /*
   * States for locate_block FSM. Note that the states for the vector_fsm
   * sub-fsm cannot be part of the locate_block hfsm, because that sub-fsm is
   * initiated from multiple states, and hfsm states can only have ONE parent
   * state.
   **/
  HFSM_STATE_DECLARE(locate_free_block_fsm, start,
                     state_machine::no_event_data);
  HFSM_STATE_DECLARE(locate_free_block_fsm, acquire_block,
                     state_machine::event_data);
  HFSM_STATE_DECLARE(locate_free_block_fsm, finished,
                     state_machine::no_event_data);

  HFSM_EXIT_DECLARE(locate_free_block_fsm, exit_acquire_block);

  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) {
  HFSM_DEFINE_STATE_MAP(state_map_ex, kSTATE_MAP) {
    HFSM_STATE_MAP_ENTRY_EX(&start, hfsm::top_state()),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&acquire_block, hfsm::top_state(),
                                    NULL,
                                    NULL, &exit_acquire_block),
        HFSM_STATE_MAP_ENTRY_EX(&finished, hfsm::top_state())
    };
  HFSM_VERIFY_STATE_MAP(state_map_ex, kSTATE_MAP);
  return &kSTATE_MAP[index];
  }

  locate_free_block_fsm(const locate_free_block_fsm& fsm) = delete;
  locate_free_block_fsm& operator=(const locate_free_block_fsm& fsm) = delete;

  /* data members */
  uint8_t m_current_state;
  uint8_t m_next_state;
  uint8_t m_initial_state;
  uint8_t m_previous_state;
  uint8_t m_last_state;

  const argos::CVector2 mc_nest_center;
  argos::CRandom::CRNG* m_rng;
  std::shared_ptr<const representation::perceived_arena_map> m_map;
  std::shared_ptr<rcppsw::common::er_server> m_server;
  vector_fsm m_vector_fsm;
  explore_fsm m_explore_fsm;
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_ACQUIRE_FREE_BLOCK_FSM_HPP_ */
