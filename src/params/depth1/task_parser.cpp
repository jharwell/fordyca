/**
 * @file task_parser.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/params/depth1/task_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth1);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void task_parser::parse(argos::TConfigurationNode &node) {
  argos::TConfigurationNode task_node = argos::GetNode(node, "task");

  m_params = rcppsw::make_unique<struct task_params>();

  argos::GetNodeAttribute(task_node,
                          "estimation_alpha",
                          m_params->tasks.estimation_alpha);
  argos::GetNodeAttribute(task_node,
                          "abort_reactivity",
                          m_params->tasks.abort_reactivity);
  argos::GetNodeAttribute(task_node,
                          "abort_offset",
                          m_params->tasks.abort_offset);
  argos::GetNodeAttribute(task_node,
                          "partition_reactivity",
                          m_params->tasks.partition_reactivity);
  argos::GetNodeAttribute(task_node,
                          "partition_offset",
                          m_params->tasks.partition_offset);
  argos::GetNodeAttribute(task_node,
                          "subtask_selection_method",
                          m_params->tasks.subtask_selection_method);
  argos::GetNodeAttribute(task_node,
                          "partition_method",
                          m_params->tasks.partition_method);
  argos::GetNodeAttribute(task_node,
                          "always_partition",
                          m_params->tasks.always_partition);
  argos::GetNodeAttribute(task_node,
                          "never_partition",
                          m_params->tasks.never_partition);

  argos::GetNodeAttribute(task_node,
                          "init_random_estimates",
                          m_params->init_random_estimates);
} /* parse() */

void task_parser::show(std::ostream &stream) {
  stream << "====================\nTask params\n====================\n";
  stream << "estimation_alpha=" << m_params->tasks.estimation_alpha
         << std::endl;
  stream << "abort_reactivity=" << m_params->tasks.abort_reactivity
         << std::endl;
  stream << "abort_offset=" << m_params->tasks.abort_offset << std::endl;
  stream << "partition_reactivity=" << m_params->tasks.partition_reactivity
         << std::endl;
  stream << "partition_offset=" << m_params->tasks.partition_offset
         << std::endl;
  stream << "subtask_selection_method="
         << m_params->tasks.subtask_selection_method << std::endl;
  stream << "partition_method=" << m_params->tasks.partition_method
         << std::endl;
  stream << "always_partition=" << m_params->tasks.always_partition
         << std::endl;
  stream << "never_partition=" << m_params->tasks.never_partition << std::endl;
  stream << "init_random_estimates=" << m_params->init_random_estimates
         << std::endl;
} /* show() */

__pure bool task_parser::validate(void) {
  if (m_params->tasks.estimation_alpha <= 0.0 ||
      m_params->tasks.abort_reactivity <= 0.0 ||
      m_params->tasks.abort_offset <= 0.0 ||
      m_params->tasks.partition_reactivity <= 0.0 ||
      m_params->tasks.partition_offset <= 0.0) {
    return false;
  }
  return true;
} /* validate() */


NS_END(depth1, params, fordyca);
