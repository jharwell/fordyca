/**
 * \file base_foraging_task.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/polymorphic_visitable.hpp"

#include "cosm/fsm/block_transporter.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics.hpp"
#include "cosm/spatial/metrics/goal_acq_metrics.hpp"
#include "cosm/ta/abort_probability.hpp"
#include "cosm/ta/logical_task.hpp"
#include "cosm/spatial/strategy/blocks/metrics/drop_metrics.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/fsm/foraging_transport_goal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \class base_foraging_task
 * \ingroup tasks
 *
 * \brief Interface specifying the visit set common to all base_foraging tasks
 * in FORDYCA, as well as common metrics reported by/on all tasks.
 */
class base_foraging_task
    : public cfsm::block_transporter<fsm::foraging_transport_goal>,
      public cfsm::metrics::block_transporter_metrics,
      public csmetrics::goal_acq_metrics,
      public cssblocks::metrics::drop_metrics {
 public:
  base_foraging_task(void) = default;
  ~base_foraging_task(void) override = default;
};

NS_END(tasks, fordyca);
