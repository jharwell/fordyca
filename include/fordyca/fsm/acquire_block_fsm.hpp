/**
 * @file acquire_block_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_ACQUIRE_BLOCK_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_ACQUIRE_BLOCK_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <utility>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>

#include "rcppsw/task_allocation/taskable.hpp"
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

namespace representation {
class perceived_arena_map;
class block;
} /* namespace representation */

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @brief The FSM for an acquiring a free (i.e. not in a cache) block in the
 * arena.
 *
 * Each robot executing this FSM will look for a block (either a known block or
 * via random exploration). Once an existing block has been acquired, it signals
 * that it has completed its task.
 */
class acquire_block_fsm : public base_foraging_fsm,
                               public  rcppsw::task_allocation::taskable {
 public:
  acquire_block_fsm(
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
    return is_vectoring() || is_exploring();
  }

  bool is_exploring(void) const {
    return (current_state() == ST_ACQUIRE_BLOCK && m_explore_fsm.is_searching()); }

  bool is_vectoring(void) const {
    return current_state() == ST_ACQUIRE_BLOCK && m_vector_fsm.in_progress();
  }
  bool is_avoiding_collision(void) const {
    return m_explore_fsm.is_avoiding_collision();
  }
  void task_execute(void) override;
  bool task_finished(void) const override { return ST_FINISHED == current_state(); }

 protected:
  enum fsm_states {
    ST_START,
    ST_ACQUIRE_BLOCK, /* superstate for finding a free block */
    ST_FINISHED,
    ST_MAX_STATES
  };

 private:
  /**
   * @brief Acquire a free block.
   *
   * @return TRUE if a block has been acquired, FALSE otherwise.
   */
  bool acquire_any_block(void);

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
  HFSM_STATE_DECLARE(acquire_block_fsm, start,
                     state_machine::no_event_data);
  HFSM_STATE_DECLARE(acquire_block_fsm, acquire_block,
                     state_machine::event_data);
  HFSM_STATE_DECLARE(acquire_block_fsm, finished,
                     state_machine::no_event_data);

  HFSM_EXIT_DECLARE(acquire_block_fsm, exit_acquire_block);

  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
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

  acquire_block_fsm(const acquire_block_fsm& fsm) = delete;
  acquire_block_fsm& operator=(const acquire_block_fsm& fsm) = delete;

  const argos::CVector2 mc_nest_center;
  argos::CRandom::CRNG* m_rng;
  std::shared_ptr<const representation::perceived_arena_map> m_map;
  std::shared_ptr<rcppsw::common::er_server> m_server;
  vector_fsm m_vector_fsm;
  explore_fsm m_explore_fsm;
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_ACQUIRE_BLOCK_FSM_HPP_ */