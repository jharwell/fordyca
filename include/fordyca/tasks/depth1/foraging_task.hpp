/**
 * @file depth1/foraging_task.hpp
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

#ifndef INCLUDE_FORDYCA_TASKS_DEPTH1_FORAGING_TASK_HPP_
#define INCLUDE_FORDYCA_TASKS_DEPTH1_FORAGING_TASK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "fordyca/tasks/base_foraging_task.hpp"
#include "rcppsw/ta/polled_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace rcppsw { namespace ta { namespace config { struct task_alloc_config; }}}
NS_START(fordyca, tasks, depth1);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @class foraging_task
 * @ingroup fordyca tasks depth1
 *
 * @brief Interface specifying the visit set for all depth1 foraging tasks
 * in FORDYCA.
 *
 * Not all tasks need all events, but it is convenient both from a design point
 * of view as well as not having to fight with the compiler as much if you do it
 * this way.
 */
class foraging_task : public base_foraging_task,
                      public rta::polled_task {
 public:
  static constexpr char kCollectorName[] = "Collector";
  static constexpr char kHarvesterName[] = "Harvester";

  foraging_task(const std::string& name,
                const struct rta::config::task_alloc_config *config,
                std::unique_ptr<rta::taskable> mechanism);
  ~foraging_task(void) override = default;

  static bool task_in_depth1(const polled_task* task) RCSW_PURE;

    /* task overrides */
  double current_time(void) const override RCSW_PURE;
};

NS_END(depth1, tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_DEPTH1_FORAGING_TASK_HPP_ */
