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
#include "rcppsw/task_allocation/abort_probability.hpp"
#include "fordyca/tasks/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks);
namespace task_allocation = rcppsw::task_allocation;

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
 * \ref collector and \ref forager. It is not abortable.
 */
class generalist : public task_allocation::partitionable_polled_task,
                   public foraging_task {
 public:
  generalist(const struct task_allocation::partitionable_task_params *params,
             std::unique_ptr<task_allocation::taskable>& mechanism);

  /* event handling */
  void accept(events::free_block_pickup &visitor) override;
  void accept(events::nest_block_drop &visitor) override;
  void accept(events::cache_block_drop &) override {}
  void accept(events::cached_block_pickup &) override {}

  /* base metrics */
  bool is_exploring_for_block(void) const override;
  bool is_avoiding_collision(void) const override;
  bool is_transporting_to_nest(void) const override;

  /* depth0 metrics */
  bool is_acquiring_block(void) const override;
  bool is_vectoring_to_block(void) const override;

  /* depth1 metrics */
  bool is_exploring_for_cache(void) const override { return false; }
  bool is_vectoring_to_cache(void) const override { return false; }
  bool is_acquiring_cache(void) const override { return false; }
  bool is_transporting_to_cache(void) const override { return false; }
  std::string task_name(void) const override { return "generalist"; }

  bool cache_acquired(void) const override { return false; }
  bool block_acquired(void) const override;

  executable_task* partition(void) override { return partitionable_task::partition(); }
  void task_start(const task_allocation::taskable_argument* const) override {}

  double current_time(void) const override;
  double calc_abort_prob(void) override;
  double calc_interface_time(double) override { return 0.0; }

 private:
  task_allocation::abort_probability m_abort_prob;
};

NS_END(tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_GENERALIST_HPP_ */
