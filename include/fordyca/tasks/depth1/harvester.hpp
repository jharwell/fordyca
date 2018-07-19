/**
 * @file harvester.hpp
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

#ifndef INCLUDE_FORDYCA_TASKS_DEPTH1_HARVESTER_HPP_
#define INCLUDE_FORDYCA_TASKS_DEPTH1_HARVESTER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/tasks/depth1/foraging_task.hpp"
#include "fordyca/tasks/depth1/existing_cache_interactor.hpp"

#include "rcppsw/patterns/visitor/visitable.hpp"
#include "rcppsw/task_allocation/abort_probability.hpp"
#include "rcppsw/task_allocation/polled_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth1);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @class harvester
 * @ingroup tasks
 *
 * @brief Task in which robots locate a free block and bring it to a known
 * cache. It is abortable, and has one task interface.
 */
class harvester : public foraging_task,
                  public existing_cache_interactor {
 public:
  harvester(const struct ta::task_params* params,
            std::unique_ptr<ta::taskable>& mechanism);

  /*
   * Event handling. This CANNOT be done using the regular visitor pattern,
   * because when visiting a \ref existing_cache_interactor, you have no way to
   * way which task the object ACTUALLY is without using a set of if()
   * statements, which is a brittle design. This is not the cleanest, but is
   * still more elegant than the alternative.
   */
  void accept(events::cache_block_drop& visitor) override;
  void accept(events::cached_block_pickup&) override {}
  void accept(events::cache_vanished& visitor) override;
  void accept(events::free_block_pickup& visitor) override;
  void accept(events::nest_block_drop&) override {}

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

  void task_start(const ta::taskable_argument*) override;
  double calc_abort_prob(void) override;
  double calc_interface_time(double start_time) override;
};

NS_END(depth1, tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_DEPTH1_HARVESTER_HPP_ */
