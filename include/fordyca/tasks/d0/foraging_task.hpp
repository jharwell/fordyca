/**
 * \file d0/foraging_task.hpp
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
#include <boost/variant.hpp>

#include "fordyca/tasks/base_foraging_task.hpp"
#include "rcppsw/patterns/visitor/polymorphic_visitable.hpp"
#include "fordyca/events/nest_interactor.hpp"
#include "fordyca/events/free_block_interactor.hpp"
#include "cosm/ta/polled_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ta::config { struct task_alloc_config; }

NS_START(fordyca, tasks, d0);
class generalist;

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \class foraging_task
 * \ingroup tasks d0
 *
 * \brief Interface specifying the visit set for all d0 foraging tasks in
 * FORDYCA.
 *
 * Not all tasks need all events, but it is convenient both from a design point
 * of view as well as not having to fight with the compiler as much if you do it
 * this way.
 */
class foraging_task
    : public base_foraging_task,
      public events::nest_interactor,
      public events::free_block_interactor,
      public cta::polled_task {
 public:
  using variant_type = boost::variant<generalist*>;

  static inline const std::string kGeneralistName = "generalist";

  foraging_task(const std::string& name,
                const cta::config::task_alloc_config* config,
                std::unique_ptr<cta::taskable> mechanism);

  ~foraging_task(void) override = default;

  static bool task_in_d0(const cta::polled_task* task) RCPPSW_PURE;
};

NS_END(d0, tasks, fordyca);
