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
#include "rcppsw/task_allocation/abort_probability.hpp"
#include "rcppsw/task_allocation/polled_task.hpp"

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
 * @class collector
 * @ingroup tasks
 *
 * @brief Task in which robots locate a cache and bring a block from it to the
 * nest. It is abortable, and has one task interface.
 */
class collector : public task_allocation::polled_task, public foraging_task {
 public:
  collector(const struct task_allocation::task_params* params,
            std::unique_ptr<task_allocation::taskable>& mechanism);

  /* event handling */
  void accept(events::cached_block_pickup& visitor) override;
  void accept(events::nest_block_drop& visitor) override;
  void accept(events::cache_vanished& visitor) override;
  void accept(events::cache_block_drop&) override {}
  void accept(events::free_block_pickup&) override {}

  /* stateless metrics */
  bool is_exploring_for_block(void) const override { return false; }
  bool is_avoiding_collision(void) const override;
  bool is_transporting_to_nest(void) const override;

  /* stateful metrics */
  bool is_acquiring_block(void) const override { return false; }
  bool is_vectoring_to_block(void) const override { return false; }

  /* depth1 metrics */
  bool is_exploring_for_cache(void) const override;
  bool is_vectoring_to_cache(void) const override;
  bool is_acquiring_cache(void) const override;
  bool is_transporting_to_cache(void) const override { return false; }

  /* task metrics */
  bool task_interface_complete(void) const override;
  double task_interface_time(void) const override;

  bool cache_acquired(void) const override;
  bool block_acquired(void) const override { return false; }

  void task_start(const task_allocation::taskable_argument*) override;
  double current_time(void) const override;
  double calc_abort_prob(void) override;
  double calc_interface_time(double start_time) override;

 private:
  // clang-format off
  bool                               m_interface_complete{false};
  mutable bool                       m_first_transport{false};
  task_allocation::abort_probability m_abort_prob;
  // clang-format on
};

NS_END(tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_COLLECTOR_HPP_ */
