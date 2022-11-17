/**
 * \file d2/foraging_task.hpp
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
#include "rcppsw/patterns/visitor/polymorphic_visitable.hpp"
#include "cosm/ta/polled_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ta::config { struct task_alloc_config; }

NS_START(fordyca, tasks, d2);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \class foraging_task
 * \ingroup tasks d2
 *
 * \brief Interface specifying the visit set for all d2 foraging tasks
 * in FORDYCA.
 *
 * Not all tasks need all events, but it is convenient both from a design point
 * of view as well as not having to fight with the compiler as much if you do it
 * this way.
 */
class foraging_task
    : public base_foraging_task,
      public cta::polled_task {

 public:
  foraging_task(const std::string& name,
                const cta::config::task_alloc_config *config,
                std::unique_ptr<cta::taskable> mechanism);
  ~foraging_task(void) override = default;

  static inline const std::string kCacheStarterName = "cache_starter";
  static inline const std::string kCacheFinisherName = "cache_finisher";
  static inline const std::string kCacheTransfererName = "cache_transferer";
  static inline const std::string kCacheCollectorName = "cache_collector";

  static bool task_in_d2(const polled_task* task) RCPPSW_PURE;

  /* task overrides */
  rtypes::timestep current_time(void) const override RCPPSW_PURE;
};

NS_END(d2, tasks, fordyca);
