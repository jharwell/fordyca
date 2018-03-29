/**
 * @file task_allocation_parser.cpp
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
#include "fordyca/params/depth1/task_allocation_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth1);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void task_allocation_parser::parse(argos::TConfigurationNode& node) {
  argos::TConfigurationNode task_node = argos::GetNode(node, "task_allocation");

  m_exec_parser.parse(argos::GetNode(task_node, "executive"));
  m_estimate_parser.parse(argos::GetNode(task_node, "init_estimates"));
  m_params = rcppsw::make_unique<struct task_allocation_params>();
  m_params->executive = *m_exec_parser.get_results();
  m_params->exec_estimates = *m_estimate_parser.get_results();
} /* parse() */

void task_allocation_parser::show(std::ostream& stream) {
  stream
      << "====================\nTask allocation params\n====================\n";
  m_exec_parser.show(stream);
  m_estimate_parser.show(stream);
} /* show() */

__pure bool task_allocation_parser::validate(void) {
  return m_exec_parser.validate() && m_estimate_parser.validate();
} /* validate() */

NS_END(depth1, params, fordyca);
