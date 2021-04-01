/**
 * \file foraging_task.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/tasks/d1/foraging_task.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"
#include "cosm/foraging/fsm/foraging_util_hfsm.hpp"
#include "cosm/ta/config/task_alloc_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, d1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_task::foraging_task(const std::string& name,
                             const struct cta::config::task_alloc_config* config,
                             std::unique_ptr<cta::taskable> mechanism)
    : polled_task(name,
                  &config->abort,
                  &config->exec_est.ema,
                  std::move(mechanism)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rtypes::timestep foraging_task::current_time(void) const {
  return dynamic_cast<cffsm::foraging_util_hfsm*>(polled_task::mechanism())
      ->sensing()
      ->tick();
} /* current_time() */

bool foraging_task::task_in_d1(const polled_task* const task) {
  return task->name() == kCollectorName || task->name() == kHarvesterName;
} /* task_in_d1() */

NS_END(d1, tasks, fordyca);
