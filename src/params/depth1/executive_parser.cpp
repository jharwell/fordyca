/**
 * @file executive_parser.cpp
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
#include "fordyca/params/depth1/executive_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth1);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void executive_parser::parse(argos::TConfigurationNode& node) {
  m_params =
      rcppsw::make_unique<struct task_allocation::partitionable_task_params>();

  argos::GetNodeAttribute(node, "estimation_alpha", m_params->estimation_alpha);
  argos::GetNodeAttribute(node, "abort_reactivity", m_params->abort_reactivity);
  argos::GetNodeAttribute(node, "abort_offset", m_params->abort_offset);
  argos::GetNodeAttribute(node,
                          "partition_reactivity",
                          m_params->partition_reactivity);
  argos::GetNodeAttribute(node, "partition_offset", m_params->partition_offset);
  argos::GetNodeAttribute(node,
                          "subtask_selection_method",
                          m_params->subtask_selection_method);
  argos::GetNodeAttribute(node, "partition_method", m_params->partition_method);
  argos::GetNodeAttribute(node, "always_partition", m_params->always_partition);
  argos::GetNodeAttribute(node, "never_partition", m_params->never_partition);
} /* parse() */

void executive_parser::show(std::ostream& stream) {
  stream << "====================\nExecutive params\n====================\n";
  stream << "estimation_alpha=" << m_params->estimation_alpha << std::endl;
  stream << "abort_reactivity=" << m_params->abort_reactivity << std::endl;
  stream << "abort_offset=" << m_params->abort_offset << std::endl;
  stream << "partition_reactivity=" << m_params->partition_reactivity
         << std::endl;
  stream << "partition_offset=" << m_params->partition_offset << std::endl;
  stream << "subtask_selection_method=" << m_params->subtask_selection_method
         << std::endl;
  stream << "partition_method=" << m_params->partition_method << std::endl;
  stream << "always_partition=" << m_params->always_partition << std::endl;
  stream << "never_partition=" << m_params->never_partition << std::endl;
} /* show() */

__pure bool executive_parser::validate(void) {
  return !(m_params->estimation_alpha <= 0.0 ||
           m_params->abort_reactivity <= 0.0 || m_params->abort_offset <= 0.0 ||
           m_params->partition_reactivity <= 0.0 ||
           m_params->partition_offset <= 0.0);
} /* validate() */

NS_END(depth1, params, fordyca);
