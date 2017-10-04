/**
 * @file explore_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_EXPLORE_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_EXPLORE_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <utility>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>

#include "fordyca/fsm/base_foraging_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace controller {
class sensor_manager;
class actuator_manager;
} /* namespace controller */

namespace state_machine = rcppsw::patterns::state_machine;

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @brief The FSM for an unpartitioned foraging task. Each robot executing this
 * FSM will locate for a block (either a known block or via random exploration),
 * pickup the block and bring it all the way back to the nest.
 */
class explore_fsm : public base_foraging_fsm {
 public:
  enum fsm_states {
    ST_START,
    ST_EXPLORE,               /* explore for stuff  */
    ST_COLLISION_AVOIDANCE,   /* Avoiding colliding with something */
    ST_NEW_DIRECTION,         /* Time to change direction during exploration */
    ST_MAX_STATES
  };

  explore_fsm(double unsuccessful_dir_change_thresh,
              const std::shared_ptr<rcppsw::common::er_server>& server,
              const std::shared_ptr<controller::sensor_manager>& sensors,
              const std::shared_ptr<controller::actuator_manager>& actuators);

  /**
   * @brief Reset the FSM
   */
  void init(void);

  /**
   * @brief Get if the robot is currently searching for something within the arena
   *
   * @return TRUE if the condition is met, FALSE otherwise.
   */
  bool is_searching(void) const { return (current_state() == ST_EXPLORE); }

  bool is_avoiding_collision(void) const {
    return current_state() == ST_COLLISION_AVOIDANCE;
  }

  /**
   * @brief Run the FSM in its current state without injecting an event into it.
   */
  void run(void);

  /**
   * @brief Reset the # of timesteps the robot has spent unsuccessfully looking
   * for a block.
   */
  void explore_time_reset(void) { m_state.time_exploring_unsuccessfully = 0; }

  /**
   * @brief Increment the # of timesteps the robot has spent unsuccessfully
   * looking for a block.
   */
  void explore_time_inc(void) { ++m_state.time_exploring_unsuccessfully; }

  /**
   * @brief Get the # of timesteps the robot has spent unsuccessfully looking
   * for a block.
   */
  size_t explore_time(void) const { return m_state.time_exploring_unsuccessfully; }

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

 private:
  /**
   * @brief Inject randomness into robot exploring by having them change their
   * direction every X timesteps if they have not yet located a block, where X
   * is set in the .argos configuration file.
   */
  struct new_direction_data : public state_machine::event_data {
    explicit new_direction_data(argos::CRadians dir_) : dir(dir_) {}
    argos::CRadians dir;
  };

  struct fsm_state {
    fsm_state(void) : time_exploring_unsuccessfully(0) {}

    size_t time_exploring_unsuccessfully;
  };

  /* inherited states */
  HFSM_STATE_INHERIT(base_foraging_fsm, collision_avoidance,
                     state_machine::event_data);
  HFSM_ENTRY_INHERIT(base_foraging_fsm, entry_collision_avoidance,
                     state_machine::no_event_data);

  /* states for exploration FSM */
  HFSM_STATE_DECLARE(explore_fsm, start, state_machine::no_event_data);
  HFSM_STATE_DECLARE(explore_fsm, new_direction, state_machine::event_data);
  HFSM_STATE_DECLARE(explore_fsm, explore, state_machine::event_data);
  HFSM_ENTRY_DECLARE(explore_fsm, entry_new_direction,
                     state_machine::no_event_data);
  HFSM_ENTRY_DECLARE(explore_fsm, entry_explore,
                     state_machine::no_event_data);

  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) {
  HFSM_DEFINE_STATE_MAP(state_map_ex, kSTATE_MAP) {
    HFSM_STATE_MAP_ENTRY_EX(&start, hfsm::top_state()),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&explore, hfsm::top_state(),
                                    NULL,
                                    &entry_explore, NULL),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&collision_avoidance, hfsm::top_state(),
                                    NULL,
                                    &entry_collision_avoidance, NULL),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&new_direction, hfsm::top_state(),
                                    NULL,
                                    &entry_new_direction, NULL),
    };
  HFSM_VERIFY_STATE_MAP(state_map_ex, kSTATE_MAP);
  return &kSTATE_MAP[index];
  }

  explore_fsm(const explore_fsm& fsm) = delete;
  explore_fsm& operator=(const explore_fsm& fsm) = delete;

  /* data members */
  const double mc_unsuccessful_dir_change;

  uint8_t m_current_state;
  uint8_t m_next_state;
  uint8_t m_initial_state;
  uint8_t m_previous_state;
  uint8_t m_last_state;

  argos::CRandom::CRNG* m_rng;
  struct fsm_state m_state;
  argos::CRadians m_new_dir;
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_EXPLORE_FSM_HPP_ */
