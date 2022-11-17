/**
 * \file foraging_task.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/tasks/d1/foraging_task.hpp"

#include "cosm/foraging/fsm/foraging_util_hfsm.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"
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
