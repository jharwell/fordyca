/**
 * @file forager.hpp
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

#ifndef INCLUDE_FORDYCA_TASKS_FORAGER_HPP_
#define INCLUDE_FORDYCA_TASKS_FORAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/task_allocation/polled_task.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/tasks/argument.hpp"
#include "fordyca/tasks/base_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks);
namespace task_allocation = rcppsw::task_allocation;

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @brief Class representing the first half of the generalist task in depth 1
 * allocation.
 */
class forager : public task_allocation::polled_task, base_task {
 public:
  forager(double alpha, std::unique_ptr<task_allocation::taskable>& mechanism) :
      polled_task("forager", alpha, mechanism) {}

  void accept(events::cache_block_drop &visitor) override;
  void accept(events::cache_found &visitor) override;
  void accept(events::free_block_pickup &visitor) override;
  void accept(events::block_found &visitor) override;

  void accept(events::cached_block_pickup &) override {};
  void accept(events::block_nest_drop &) override {};

  executable_task* partition(void) override { return nullptr; }
  double abort_prob(void) override { return 0.0; }
  void task_start(__unused const task_allocation::taskable_argument* const arg) override {
    foraging_signal_argument a(controller::foraging_signal::ACQUIRE_FREE_BLOCK);
    task_allocation::polled_task::mechanism()->task_start(&a);
}
  double calc_elapsed_time(double exec_time) const override;
};

NS_END(tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_FORAGER_HPP_ */
