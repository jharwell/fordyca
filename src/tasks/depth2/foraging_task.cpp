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
#include "fordyca/tasks/depth2/foraging_task.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "rcppsw/ta/config/task_alloc_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_task::foraging_task(const std::string& name,
                             const rta::config::task_alloc_config* config,
                             std::unique_ptr<rta::taskable> mechanism)
    : polled_task(name,
                  &config->abort,
                  &config->exec_est.ema,
                  std::move(mechanism)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rtypes::timestep foraging_task::current_time(void) const {
  return dynamic_cast<fsm::base_foraging_fsm*>(polled_task::mechanism())
      ->sensors()
      ->tick();
} /* current_time() */

bool foraging_task::task_in_depth2(const polled_task* const task) {
  return task->name() == kCacheStarterName ||
         task->name() == kCacheFinisherName ||
         task->name() == kCacheTransfererName ||
         task->name() == kCacheCollectorName;
} /* task_in_depth2() */

NS_END(depth2, tasks, fordyca);
