/**
 * @file cache_finisher.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_TASKS_DEPTH2_CACHE_FINISHER_HPP_
#define INCLUDE_FORDYCA_TASKS_DEPTH2_CACHE_FINISHER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/tasks/depth2/foraging_task.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/tasks/depth2/new_cache_interactor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth2);

namespace task_allocation = rcppsw::task_allocation;

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @class cache_finisher
 * @ingroup tasks depth2
 *
 * @brief Task in which robots locate a free block and drop it next to/on top of
 * a free block in the arena to finish the creation of a new cache. It is
 * abortable, and has one task interface.
 */
class cache_finisher : public foraging_task,
                       public new_cache_interactor {
 public:
  cache_finisher(const struct task_allocation::task_params* params,
            std::unique_ptr<task_allocation::taskable>& mechanism);

  /*
   * Event handling. This CANNOT be done using the regular visitor pattern,
   * because when visiting a \ref new_cache_interactor, you have no way to way
   * which depth2 task the object ACTUALLY is without using a set of if()
   * statements, which is a brittle design. This is not the cleanest, but is
   * still more elegant than the alternative.
   */
  void accept(events::free_block_drop& visitor) override;

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
  bool task_at_interface(void) const override;
  double task_last_exec_time(void) const override { return last_exec_time(); }
  double task_last_interface_time(void) const override { return last_interface_time(); }
  bool task_completed(void) const override { return task_finished(); }
  bool task_aborted(void) const override { return executable_task::task_aborted(); }

  void task_start(const task_allocation::taskable_argument*) override;
  double calc_abort_prob(void) override;
  double calc_interface_time(double start_time) override;
};

NS_END(depth2, tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_DEPTH2_CACHE_FINISHER_HPP_ */
