/**
 * \file base_foraging_task.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_TASKS_BASE_FORAGING_TASK_HPP_
#define INCLUDE_FORDYCA_TASKS_BASE_FORAGING_TASK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/polymorphic_visitable.hpp"

#include "cosm/fsm/metrics/goal_acq_metrics.hpp"
#include "cosm/ta/abort_probability.hpp"
#include "cosm/ta/logical_task.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/fsm/block_transporter.hpp"

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
class base_foraging_task : public fsm::block_transporter,
                           public cfmetrics::goal_acq_metrics {
 public:
  base_foraging_task(void) = default;
  ~base_foraging_task(void) override = default;
};

NS_END(tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_BASE_FORAGING_TASK_HPP_ */
