/**
 * @file foraging_task.hpp
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

#ifndef INCLUDE_FORDYCA_TASKS_DEPTH2_FORAGING_TASK_HPP_
#define INCLUDE_FORDYCA_TASKS_DEPTH2_FORAGING_TASK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/tasks/base_foraging_task.hpp"
#include "rcppsw/patterns/visitor/polymorphic_visitable.hpp"
#include "rcppsw/task_allocation/polled_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace events {
class free_block_drop;
} // namespace events

namespace visitor = rcppsw::patterns::visitor;

NS_START(tasks, depth2);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @class foraging_task
 * @ingroup tasks depth2
 *
 * @brief Interface specifying the visit set for all depth2 foraging tasks
 * in FORDYCA.
 *
 * Not all tasks need all events, but it is convenient both from a design point
 * of view as well as not having to fight with the compiler as much if you do it
 * this way.
 */
class foraging_task
    : public base_foraging_task,
      public ta::polled_task,
      public visitor::polymorphic_accept_set<events::free_block_drop> {
 public:
  foraging_task(const std::string& name,
                const struct ta::task_params *params,
                std::unique_ptr<ta::taskable>& mechanism);

  static constexpr char kCacheStarterName[] = "Cache Starter";
  static constexpr char kCacheFinisherName[] = "Cache Finisher";
  static constexpr char kCacheTransfererName[] = "Cache Transferer";

  /* task overrides */
  double current_time(void) const override;

 protected:
  void interface_complete(bool interface_complete) { m_interface_complete = interface_complete; }
  bool interface_complete(void) const { return m_interface_complete; }

 private:
  bool m_interface_complete{false};
};

NS_END(depth2, tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_DEPTH2_FORAGING_TASK_HPP_ */
