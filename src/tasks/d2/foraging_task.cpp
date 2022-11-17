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
#include "fordyca/tasks/d2/foraging_task.hpp"

#include "cosm/spatial/fsm/util_hfsm.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"
#include "cosm/ta/config/task_alloc_config.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, d2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_task::foraging_task(const std::string& name,
                             const cta::config::task_alloc_config* config,
                             std::unique_ptr<cta::taskable> mechanism)
    : polled_task(name,
                  &config->abort,
                  &config->exec_est.ema,
                  std::move(mechanism)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rtypes::timestep foraging_task::current_time(void) const {
  return dynamic_cast<csfsm::util_hfsm*>(polled_task::mechanism())
      ->sensing()
      ->tick();
} /* current_time() */

bool foraging_task::task_in_d2(const cta::polled_task* const task) {
  return task->name() == kCacheStarterName ||
         task->name() == kCacheFinisherName ||
         task->name() == kCacheTransfererName ||
         task->name() == kCacheCollectorName;
} /* task_in_d2() */

NS_END(d2, tasks, fordyca);
