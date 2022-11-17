/**
 * \file d1/foraging_task.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "fordyca/tasks/base_foraging_task.hpp"
#include "cosm/ta/polled_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ta::config { struct task_alloc_config; }

NS_START(fordyca, tasks, d1);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \class foraging_task
 * \ingroup tasks d1
 *
 * \brief Interface specifying the visit set for all d1 foraging tasks
 * in FORDYCA.
 *
 * Not all tasks need all events, but it is convenient both from a design point
 * of view as well as not having to fight with the compiler as much if you do it
 * this way.
 */
class foraging_task : public base_foraging_task,
                      public cta::polled_task {
 public:
  static inline const std::string kCollectorName = "collector";
  static inline const std::string kHarvesterName = "harvester";

  foraging_task(const std::string& name,
                const struct cta::config::task_alloc_config *config,
                std::unique_ptr<cta::taskable> mechanism);
  ~foraging_task(void) override = default;

  static bool task_in_d1(const polled_task* task) RCPPSW_PURE;

  /* task overrides */
  rtypes::timestep current_time(void) const override RCPPSW_PURE;
};

NS_END(d1, tasks, fordyca);
