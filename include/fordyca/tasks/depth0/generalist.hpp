/**
 * @file generalist.hpp
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

#ifndef INCLUDE_FORDYCA_TASKS_DEPTH0_GENERALIST_HPP_
#define INCLUDE_FORDYCA_TASKS_DEPTH0_GENERALIST_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/tasks/depth0/foraging_task.hpp"
#include "rcppsw/task_allocation/partitionable_polled_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth0);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @class generalist
 * @ingroup tasks
 *
 * @brief Class representing depth 0 task allocation: Perform the whole foraging
 * task: (1) Find a free block, and (2) bring it to the nest.
 *
 * It is decomposable into two subtasks that result in the same net change to
 * the arena state when run in sequence (possibly by two different robots):
 * \ref collector and \ref forager. It is not abortable at task interfaces,
 * because it does not have any, but it IS still abortable if its current
 * execution time takes too long (as configured by parameters).
 */
class generalist : public ta::partitionable_polled_task,
                   public foraging_task {
 public:
  generalist(const struct ta::partitionable_task_params* params,
             std::unique_ptr<ta::taskable>& mechanism);

  /* event handling */
  void accept(events::free_block_pickup& visitor) override;
  void accept(events::nest_block_drop& visitor) override;

  /* base FSM metrics */
  TASK_WRAPPER_DECLARE(bool, is_avoiding_collision);

  /* goal acquisition metrics */
  TASK_WRAPPER_DECLARE(bool, goal_acquired);
  TASK_WRAPPER_DECLARE(bool, is_exploring_for_goal);
  TASK_WRAPPER_DECLARE(bool, is_vectoring_to_goal);
  TASK_WRAPPER_DECLARE(acquisition_goal_type, acquisition_goal);

  /* block transportation */
  TASK_WRAPPER_DECLARE(transport_goal_type, block_transport_goal);

  /* task metrics */
  bool at_interface(void) const override { return false; }

  void task_start(const ta::taskable_argument* const) override {}

  double current_time(void) const override;
  double calc_abort_prob(void) override;
  double calc_interface_time(double) override { return 0.0; }
};

NS_END(depth0, tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_DEPTH0_GENERALIST_HPP_ */
