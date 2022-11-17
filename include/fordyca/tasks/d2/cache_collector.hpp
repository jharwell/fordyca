/**
 * \file cache_collector.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>
#include <memory>

#include "fordyca/tasks/d1/collector.hpp"
#include "fordyca/tasks/d2/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, d2);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \class cache_collector
 * \ingroup tasks d2
 *
 * \brief Task in which robots locate a cache and bring a block from it to the
 * nest. It is abortable, and has one task interface.
 */
class cache_collector final : public d1::collector {
 public:
  cache_collector(const cta::config::task_alloc_config* config,
                  std::unique_ptr<cta::taskable> mechanism) :
      collector(config,
                d2::foraging_task::kCacheCollectorName,
                std::move(mechanism)) {}
};

NS_END(d2, tasks, fordyca);
