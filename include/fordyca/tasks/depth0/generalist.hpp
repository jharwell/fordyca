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

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth0);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @class generalist
 * @ingroup fordyca tasks
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
class generalist : public foraging_task {
 public:
  generalist(const rta::task_alloc_params* params,
             std::unique_ptr<rta::taskable> mechanism);

  /* event handling */
  void accept(events::detail::free_block_pickup& visitor) override;
  void accept(events::detail::free_block_drop&) override {}
  void accept(events::detail::nest_block_drop& visitor) override;
  void accept(events::detail::block_vanished& visitor) override;

  /* goal acquisition metrics */
  TASK_WRAPPER_DECLAREC(bool, goal_acquired);
  TASK_WRAPPER_DECLAREC(bool, is_exploring_for_goal);
  TASK_WRAPPER_DECLAREC(bool, is_vectoring_to_goal);
  TASK_WRAPPER_DECLAREC(acquisition_goal_type, acquisition_goal);
  TASK_WRAPPER_DECLAREC(rmath::vector2u, acquisition_loc);

  /* block transportation */
  TASK_WRAPPER_DECLAREC(transport_goal_type, block_transport_goal);

  /* task metrics */
  bool task_at_interface(void) const override { return false; }
  bool task_completed(void) const override { return task_finished(); }

  void task_start(const rta::taskable_argument* const) override {}

  double current_time(void) const override;
  double interface_time_calc(uint, double) override { return 0.0; }
  void active_interface_update(int) override {}
  double abort_prob_calc(void) override;
};

NS_END(depth0, tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_DEPTH0_GENERALIST_HPP_ */
