/**
 * @file base_explore_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_BASE_EXPLORE_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_BASE_EXPLORE_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>

#include "rcppsw/task_allocation/taskable.hpp"
#include "fordyca/fsm/base_foraging_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace controller { class base_foraging_sensors; class actuator_manager; }
namespace state_machine = rcppsw::patterns::state_machine;
namespace task_allocation = rcppsw::task_allocation;

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class base_explore_fsm
 *
 * @brief The base FSM for an exploration subtask. Does not actually contain an
 * FSM per-se, but just some pieces common to all exploration FSMs.
 *
 * This class cannot be instantiated on its own.
 */
class base_explore_fsm : public base_foraging_fsm,
                         public task_allocation::taskable {
 public:
  base_explore_fsm(double unsuccessful_dir_change_thresh,
                   const std::shared_ptr<rcppsw::er::server>& server,
                   const std::shared_ptr<controller::base_foraging_sensors>& sensors,
                   const std::shared_ptr<controller::actuator_manager>& actuators,
                   uint8_t max_states);

  base_explore_fsm(const base_explore_fsm& fsm) = delete;
  base_explore_fsm& operator=(const base_explore_fsm& fsm) = delete;

  /* taskable overrides */
  void task_start( __unused const rcppsw::task_allocation::taskable_argument*) override {}

  /**
   * @brief Reset the FSM
   */
  void init(void) override;

  /**
   * @brief Run the FSM in its current state without injecting an event into it.
   */
  void run(void);

 protected:
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
  struct fsm_state {
    size_t time_exploring_unsuccessfully{0};
  };

  /**
   * @brief Robots entering this state will randomly change their exploration
   * direction to the specified direction. All signals are ignored in this
   * state. Once the direction change has been accomplished, the robot will
   * transition back to \enum fsm_states::ST_EXPLORE.
   */
  HFSM_STATE_DECLARE(base_explore_fsm, new_direction, state_machine::event_data);

  /**
   * @brief Simple state for entry into the new direction state, used to change
   * LED color for visualization purposes.
   */
  HFSM_ENTRY_DECLARE_ND(base_explore_fsm, entry_new_direction);

  /**
   * @brief Simple state for entry in the main exploration state, used to change
   * LED color for visualization purposes.
   */
  HFSM_ENTRY_DECLARE_ND(base_explore_fsm, entry_explore);

  struct fsm_state      m_state;
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_BASE_EXPLORE_FSM_HPP_ */
