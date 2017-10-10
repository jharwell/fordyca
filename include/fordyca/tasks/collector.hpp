/**
 * @file collector.hpp
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

#ifndef INCLUDE_FORDYCA_TASKS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_TASKS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/task_allocation/atomic_polled_task.hpp"
#include "fordyca/tasks/argument.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks);
namespace task_allocation = rcppsw::task_allocation;

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @brief Class representing the second half of the generalist task in depth 1
 * allocation.
 */
class collector : public task_allocation::atomic_polled_task {
 public:
  collector(double alpha, task_allocation::taskable * const taskable) :
      atomic_polled_task("collector", alpha, taskable) {}

  void task_start(__unused const task_allocation::taskable_argument* const arg) {
    foraging_signal_argument a(controller::foraging_signal::ACQUIRE_CACHED_BLOCK);
    task_allocation::atomic_polled_task::mechanism()->task_start(&a);
  }
};


NS_END(tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_COLLECTOR_HPP_ */
