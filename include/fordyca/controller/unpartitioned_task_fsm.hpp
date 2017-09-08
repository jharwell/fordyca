/**
 * @file unpartitioned_task_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_UNPARTITIONED_TASK_FSM_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_UNPARTITIONED_TASK_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <utility>

#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>
#include "rcppsw/patterns/state_machine/hfsm.hpp"
#include "fordyca/params/params.hpp"
#include "fordyca/controller/sensor_manager.hpp"
#include "fordyca/controller/actuator_manager.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/controller/vector_to_goal.hpp"
#include "fordyca/controller/random_foraging_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
namespace fsm = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief Signals that sub-states can return in order to notify their super
 * states that a conditiot that they do not know how to handle has arised.
 */
class foraging_signal : public fsm::event_signal {
 public:
  enum {
    BLOCK_LOCATED = fsm::event_signal::EXTERNAL_SIGNALS,
    BLOCK_ACQUIRED,
    ARRIVED_AT_TARGET
  };
};

/**
 * @brief The FSM for an unpartitioned foraging task. Each robot executing this
 * FSM will locate for a block (either a known block or via random exploration),
 * pickup the block and bring it all the way back to the nest.
 */
class unpartitioned_task_fsm : public random_foraging_fsm {
 public:
  unpartitioned_task_fsm(const struct foraging_fsm_params* params,
               const std::shared_ptr<rcppsw::common::er_server>& server,
               const std::shared_ptr<sensor_manager>& sensors,
               const std::shared_ptr<actuator_manager>& actuators,
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
  bool is_searching_for_block(void) {
    return current_state() == ST_EXPLORE ||
        (current_state() == ST_LOCATE_BLOCK && !m_vector_fsm.in_progress());
  }
  /**
   * @brief Pass a block fround event from the controller's/robot's sensors to
   * the FSM, so that it can update state accordingly.
   */
  void event_block_acquired(void);
  void event_block_located(void);

  void run(void) { generated_event(true); state_engine(); }

 protected:
  enum fsm_states {
    ST_LOCATE_BLOCK = random_foraging_fsm::ST_MAX_STATES, /* superstate for finding a block */
    ST_MAX_STATES
  };

 private:
  /* member functions */
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
   * @brief Acquire a known block by calculating the "best" block according to a
   * utility equation, and then traveling to that location and attempting to
   * pickup the block at that location.
   *
   * @return TRUE if a block has been acquired, FALSE otherwise.
   */
  bool acquire_block(void);

  /**
   * @brief Acquire a known block.
   */
  void acquire_known_block(
      std::list<std::pair<const representation::block*, double>> blocks);

 private:
  /* non-hierarchical states */
  HFSM_STATE_INHERIT(random_foraging_fsm, start, fsm::no_event_data);
  HFSM_STATE_INHERIT(random_foraging_fsm, return_to_nest, fsm::no_event_data);
  HFSM_STATE_INHERIT(random_foraging_fsm, leaving_nest, fsm::no_event_data);

  HFSM_ENTRY_INHERIT(random_foraging_fsm, entry_return_to_nest,
                     fsm::no_event_data);
  HFSM_ENTRY_INHERIT(random_foraging_fsm, entry_leaving_nest,
                     fsm::no_event_data);

  HFSM_EXIT_INHERIT(random_foraging_fsm, exit_leaving_nest);

  /*
   * States for locate_block FSM. Note that the states for the
   * vector_to_goal sub-fsm cannot be part of the locate_block hfsm, because that
   * sub-fsm is initiated from multiple states, and hfsm states can only have
   * ONE parent state.
   **/
  HFSM_STATE_DECLARE(unpartitioned_task_fsm, locate_block, fsm::event_data);

  /*
   * States for exploration sub-fsm (part of locate_block fsm).
   */
  HFSM_STATE_INHERIT(random_foraging_fsm, new_direction, new_direction_data);
  HFSM_STATE_DECLARE(unpartitioned_task_fsm, explore, fsm::event_data);
  HFSM_STATE_INHERIT(random_foraging_fsm, collision_avoidance,
                     fsm::event_data);

  HFSM_ENTRY_INHERIT(random_foraging_fsm, entry_new_direction,
                     fsm::no_event_data);
  HFSM_ENTRY_INHERIT(random_foraging_fsm, entry_explore, fsm::no_event_data);
  HFSM_ENTRY_INHERIT(random_foraging_fsm, entry_collision_avoidance,
                     fsm::no_event_data);

  HFSM_EXIT_DECLARE(unpartitioned_task_fsm, exit_locate_block);

  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) {
  HFSM_DEFINE_STATE_MAP(state_map_ex, kSTATE_MAP) {
    HFSM_STATE_MAP_ENTRY_EX(&start, hfsm::top_state()),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&explore, &locate_block,
                                    NULL,
                                    &entry_explore, NULL),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&new_direction, &locate_block,
                                    NULL,
                                   &entry_new_direction, NULL),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&return_to_nest, hfsm::top_state(),
                                    NULL,
                                   &entry_return_to_nest, NULL),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&leaving_nest, hfsm::top_state(),
                                    NULL,
                                   &entry_leaving_nest, &exit_leaving_nest),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&collision_avoidance, &locate_block,
                                    NULL,
                                    &entry_collision_avoidance, NULL),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&locate_block, hfsm::top_state(),
                                    NULL,
                                    NULL, &exit_locate_block)
    };
  HFSM_VERIFY_STATE_MAP(state_map_ex, kSTATE_MAP);
  return &kSTATE_MAP[index];
  }

  unpartitioned_task_fsm(const unpartitioned_task_fsm& fsm) = delete;
  unpartitioned_task_fsm& operator=(const unpartitioned_task_fsm& fsm) = delete;

  /* data members */
  uint8_t m_current_state;
  uint8_t m_next_state;
  uint8_t m_initial_state;
  uint8_t m_previous_state;
  uint8_t m_last_state;

  argos::CRandom::CRNG* m_rng;
  struct fsm_state m_state;
  std::shared_ptr<const struct foraging_fsm_params> mc_params;
  std::shared_ptr<sensor_manager> m_sensors;
  std::shared_ptr<actuator_manager> m_actuators;
  std::shared_ptr<const representation::perceived_arena_map> m_map;
  std::shared_ptr<rcppsw::common::er_server> m_server;
  vector_to_goal m_vector_fsm;
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_UNPARTITIONED_TASK_FSM_HPP_ */
