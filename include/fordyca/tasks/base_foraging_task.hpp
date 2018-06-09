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
#include <string>

#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/tasks/argument.hpp"
#include "rcppsw/metrics/tasks/execution_metrics.hpp"
#include "rcppsw/patterns/visitor/polymorphic_visitable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks);
namespace visitor = rcppsw::patterns::visitor;
using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using transport_goal_type = fsm::block_transporter::goal_type;

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
class base_foraging_task
    : public rcppsw::metrics::tasks::execution_metrics,
      public metrics::fsm::goal_acquisition_metrics,
      public fsm::block_transporter {

 public:
  explicit base_foraging_task(const std::string& name) : mc_name(name) {}

  std::string name(void) const { return mc_name; }

 private:
  const std::string mc_name;
};

NS_END(tasks, fordyca);

#endif /* INCLUDE_FORDYCA_TASKS_BASE_FORAGING_TASK_HPP_ */
