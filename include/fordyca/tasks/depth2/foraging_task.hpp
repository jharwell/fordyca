/**
 * @file depth2/foraging_task.hpp
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
#include <memory>

#include "fordyca/tasks/base_foraging_task.hpp"
#include "rcppsw/patterns/visitor/polymorphic_visitable.hpp"
#include "rcppsw/ta/polled_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace rcppsw { namespace ta { namespace config { struct task_alloc_config; }}}
NS_START(fordyca, tasks, depth2);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @class foraging_task
 * @ingroup fordyca tasks depth2
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
      public rta::polled_task {
 public:
  foraging_task(const std::string& name,
                const rta::config::task_alloc_config *config,
                std::unique_ptr<rta::taskable> mechanism);
  ~foraging_task(void) override = default;

  static constexpr char kCacheStarterName[] = "Cache Starter";
  static constexpr char kCacheFinisherName[] = "Cache Finisher";
  static constexpr char kCacheTransfererName[] = "Cache Transferer";
  static constexpr char kCacheCollectorName[] = "Cache Collector";

  static bool task_in_depth2(const polled_task* task) RCSW_PURE;

  /* task overrides */
  rtypes::timestep current_time(void) const override RCSW_PURE;
};

NS_END(depth2, tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_DEPTH2_FORAGING_TASK_HPP_ */
