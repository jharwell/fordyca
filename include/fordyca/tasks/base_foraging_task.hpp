/**
 * @file base_foraging_task.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "rcppsw/metrics/tasks/execution_metrics.hpp"
#include "rcppsw/patterns/visitor/polymorphic_visitable.hpp"
#include "rcppsw/task_allocation/abort_probability.hpp"
#include "fordyca/metrics/fsm/base_fsm_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks);
namespace visitor = rcppsw::patterns::visitor;
using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using transport_goal_type = fsm::block_transporter::goal_type;
namespace ta = rcppsw::task_allocation;

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @class base_foraging_task
 * @ingroup tasks
 *
 * @brief Interface specifying the visit set common to all base_foraging tasks
 * in FORDYCA, as well as common metrics reported by/on all tasks.
 */
class base_foraging_task : public virtual rcppsw::metrics::tasks::execution_metrics,
                           public virtual metrics::fsm::base_fsm_metrics,
                           public virtual metrics::fsm::goal_acquisition_metrics,
                           public fsm::block_transporter {
 public:
  explicit base_foraging_task(const struct ta::abort_params* params);

  ta::abort_probability abort_prob(void) const { return m_abort_prob; }

 private:
  ta::abort_probability m_abort_prob;
};

NS_END(tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_BASE_FORAGING_TASK_HPP_ */
