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
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>

#include "fordyca/fsm/base_foraging_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace controller {
class base_foraging_sensors;
class actuator_manager;
} /* namespace controller */

namespace state_machine = rcppsw::patterns::state_machine;

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class explore_fsm
 *
 * @brief The FSM for an exploration subtask. Each robot executing this FSM will
 * roam around randomly until it finds a block (or a cache), and then signal a
 * higher level FSM that it has done so.
 *
 * Note that this FSM CANNOT be taskable, as that would require it to pick which
 * type of object it signals that it has found for task completion: block or
 * cache. Much easier/better to leave that to higher level FSMs.
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
              const std::shared_ptr<controller::base_foraging_sensors>& sensors,
              const std::shared_ptr<controller::actuator_manager>& actuators);

  /**
   * @brief Reset the FSM
   */
  void init(void) override;

  /**
   * @brief Get if the robot is currently searching for something within the arena
   *
   * @return \c TRUE if the condition is met, \c FALSE otherwise.
   */
  bool is_searching(void) const { return (current_state() == ST_EXPLORE); }

  /**
   * @brief Get if the robot is currently engaged in collision avoidance.
   *
   *
   * @return \c TRUE if the condition is met, \c FALSE otherwise.
   */
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
  HFSM_STATE_INHERIT_ND(base_foraging_fsm, collision_avoidance);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_collision_avoidance);

  /**
   * @brief Starting/reset state for FSM. Has no purpose other than that.
   */
  HFSM_STATE_DECLARE_ND(explore_fsm, start);

  /**
   * @brief Robots entering this state will randomly change their exploration
   * direction to the specified direction. All signals are ignored in this
   * state. Once the direction change has been accomplished, the robot will
   * transition back to \enum fsm_states::ST_EXPLORE.
   */
  HFSM_STATE_DECLARE(explore_fsm, new_direction, state_machine::event_data);

  /**
   * @brief The main state for the explore FSM. Robots in this state maintain
   * their heading, looking for SOMETHING of interest, until they find it or
   * exceed the direction change threshold.
   *
   * This state MUST have a parent state defined that is not \ref hfsm::top_state().
   *
   * Things that can be searched for via exploring are:
   *
   * - blocks -> \ref foraging_signal::BLOCK_LOCATED returned when found
   * - caches -> \ref foraging_signal::CACHED_LOCATED returned when found
   *
   * Once something is found, this FSM has no way of knowing whether its parent
   * FSM is interested in it or not, so it will return one of the signals above
   * whenever it finds anything, and leave it up to the higher level to decide
   * what, if anything, to do.
   */
  HFSM_STATE_DECLARE_ND(explore_fsm, explore);

  /**
   * @brief Simple state for entry into the new direction state, used to change
   * LED color for visualization purposes.
   */
  HFSM_ENTRY_DECLARE_ND(explore_fsm, entry_new_direction);

  /**
   * @brief Simple state for entry in the main exploration state, used to change
   * LED color for visualization purposes.
   */
  HFSM_ENTRY_DECLARE_ND(explore_fsm, entry_explore);

  /**
   * @brief Defines the state map for the FSM. Note that the order of the states
   * in the map MUST match the order of the states in \enum fsm_states, or
   * things will not work correctly.
   */
  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
    return &mc_state_map[index];
  }

  explore_fsm(const explore_fsm& fsm) = delete;
  explore_fsm& operator=(const explore_fsm& fsm) = delete;

  const double mc_unsuccessful_dir_change;

  argos::CRandom::CRNG* m_rng;
  struct fsm_state m_state;
  argos::CRadians m_new_dir;
  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_EXPLORE_FSM_HPP_ */
