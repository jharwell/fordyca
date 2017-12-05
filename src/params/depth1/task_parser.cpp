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
#include "rcppsw/utils/line_parser.hpp"
#include "fordyca/params/depth1/task_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth1);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void task_parser::parse(argos::TConfigurationNode& node) {
  argos::TConfigurationNode task_node = argos::GetNode(node, "task");

  m_params.reset(new task_allocation::partitionable_task_params);

  argos::GetNodeAttribute(task_node, "estimation_alpha",
                          m_params->estimation_alpha);
  argos::GetNodeAttribute(task_node, "abort_reactivity", m_params->abort_reactivity);
  argos::GetNodeAttribute(task_node, "abort_offset", m_params->abort_offset);
  argos::GetNodeAttribute(task_node, "partition_reactivity", m_params->partition_reactivity);
  argos::GetNodeAttribute(task_node, "partition_offset", m_params->partition_offset);
  argos::GetNodeAttribute(task_node, "proportionality_estimate",
                          m_params->proportionality_estimate);
  argos::GetNodeAttribute(task_node, "subtask_selection_method",
                          m_params->subtask_selection_method);
  argos::GetNodeAttribute(task_node, "partition_method",
                          m_params->partition_method);
} /* parse() */

void task_parser::show(std::ostream& stream) {
  stream << "====================\nTASK params\n====================\n";
  stream << "estimation_alpha=" << m_params->estimation_alpha << std::endl;
  stream << "abort_reactivity=" << m_params->abort_reactivity << std::endl;
  stream << "abort_offset=" << m_params->abort_offset << std::endl;
  stream << "partition_reactivity=" << m_params->partition_reactivity << std::endl;
  stream << "partition_offset=" << m_params->partition_offset << std::endl;
  stream << "proportionality_estimate=" << m_params->proportionality_estimate << std::endl;
  stream << "subtask_selection_method=" << m_params->subtask_selection_method << std::endl;
  stream << "partition_method=" << m_params->partition_method << std::endl;
} /* show() */

NS_END(depth1, params, fordyca);
