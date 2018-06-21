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
#include "fordyca/controller/base_sensing_subsystem.hpp"
#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "rcppsw/task_allocation/task_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth2);

/*******************************************************************************
 * Constant Definitions
 ******************************************************************************/
constexpr char foraging_task::kCacheStarterName[];
constexpr char foraging_task::kCacheFinisherName[];
constexpr char foraging_task::kCacheTransfererName[];

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_task::foraging_task(const std::string& name,
                             const struct ta::task_params* params,
                             std::unique_ptr<ta::taskable>& mechanism)
    : base_foraging_task(&params->abort),
      polled_task(name, params, std::move(mechanism)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
__rcsw_pure double foraging_task::current_time(void) const {
  return dynamic_cast<fsm::base_foraging_fsm*>(polled_task::mechanism())
      ->base_sensors()
      ->tick();
} /* current_time() */

NS_END(depth2, tasks, fordyca);
