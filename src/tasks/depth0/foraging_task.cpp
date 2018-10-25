/**
 * @file foraging_task.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/tasks/depth0/foraging_task.hpp"
#include "rcppsw/task_allocation/task_allocation_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_task::foraging_task(const std::string& name,
                             const ta::task_allocation_params* const params,
                             std::unique_ptr<ta::taskable> mechanism) :
    polled_task(name,
                &params->abort,
                &params->exec_est.ema,
                std::move(mechanism)) {}

/*******************************************************************************
 * Constant Definitions
 ******************************************************************************/
constexpr char foraging_task::kGeneralistName[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool foraging_task::task_in_depth0(const ta::polled_task* const task) {
  return task->name() == kGeneralistName;
} /* task_in_depth0() */

NS_END(depth0, tasks, fordyca);
