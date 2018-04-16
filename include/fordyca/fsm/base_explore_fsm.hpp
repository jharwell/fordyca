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

#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "rcppsw/task_allocation/taskable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace state_machine = rcppsw::patterns::state_machine;
namespace task_allocation = rcppsw::task_allocation;

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class base_explore_fsm
 * @ingroup fsm
 *
 * @brief The base FSM for an exploration subtask. Does not actually contain an
 * FSM per-se, but just some pieces common to all exploration FSMs.
 *
 * This class cannot be instantiated on its own.
 */
class base_explore_fsm : public base_foraging_fsm,
                         public task_allocation::taskable {
 public:
  base_explore_fsm(
      const std::shared_ptr<rcppsw::er::server>& server,
      const std::shared_ptr<controller::saa_subsystem>& saa,
      uint8_t max_states);

  base_explore_fsm(const base_explore_fsm& fsm) = delete;
  base_explore_fsm& operator=(const base_explore_fsm& fsm) = delete;

  /* taskable overrides */
  void task_start(const rcppsw::task_allocation::taskable_argument*) override {}
  void task_execute(void) override;

  /**
   * @brief Run the FSM in its current state without injecting an event into it.
   */
  void run(void);

  /**
   * @brief Get if the robot is currently engaged in collision avoidance.
   *
   * @return \c TRUE if the condition is met, \c FALSE otherwise.
   */
  bool is_avoiding_collision(void) const;

 protected:
  /**
   * @brief Perform random walk exploration: wander force + avoidance force.
   */
  void random_explore(void);

 private:
  /**
   * @brief Simple state for entry in the main exploration state, used to change
   * LED color for visualization purposes.
   */
  HFSM_ENTRY_DECLARE_ND(base_explore_fsm, entry_explore);
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_BASE_EXPLORE_FSM_HPP_ */
