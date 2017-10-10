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

#ifndef INCLUDE_FORDYCA_TASKS_GENERALIST_HPP_
#define INCLUDE_FORDYCA_TASKS_GENERALIST_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/task_allocation/partitionable_polled_task.hpp"
#include "rcppsw/task_allocation/atomic_polled_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks);
namespace task_allocation = rcppsw::task_allocation;

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @brief Class representing depth 0 task allocation: Perform the whole foraging
 * task: (1) Find a block block, and (2) bring it to the nest.
 *
 * It is decomposable into two subtasks that result in the same net change to
 * the arena state when run in sequence (possibly by two different robots).
 */
class generalist : public task_allocation::partitionable_polled_task<task_allocation::atomic_polled_task,
                                                              task_allocation::atomic_polled_task> {
 public:
  generalist(double alpha, double reactivity,
             double abort_offset, task_allocation::taskable* taskable) :
      partitionable_polled_task("generalist", alpha, reactivity, abort_offset,
                                taskable) {}
};

NS_END(tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_GENERALIST_HPP_ */
