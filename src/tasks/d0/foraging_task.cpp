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
#include "fordyca/tasks/d0/foraging_task.hpp"

#include "cosm/ta/config/task_alloc_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, d0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_task::foraging_task(const std::string& name,
                             const cta::config::task_alloc_config* const config,
                             std::unique_ptr<cta::taskable> mechanism)
    : polled_task(name,
                  &config->abort,
                  &config->exec_est.ema,
                  std::move(mechanism)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool foraging_task::task_in_d0(const cta::polled_task* const task) {
  return task->name() == kGeneralistName;
} /* task_in_d0() */

NS_END(d0, tasks, fordyca);
