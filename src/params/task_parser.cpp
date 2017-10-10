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
#include "fordyca/params/task_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void task_parser::parse(argos::TConfigurationNode& node) {
  argos::TConfigurationNode task_node = argos::GetNode(node, "task");

  m_params.reset(new task_params);

  argos::GetNodeAttribute(task_node, "estimation_alpha",
                          m_params->estimation_alpha);
  argos::GetNodeAttribute(task_node, "reactivity", m_params->reactivity);
} /* parse() */

void task_parser::show(std::ostream& stream) {
  stream << "====================\nTASK params\n====================\n";
  stream << "estimation_alpha=" << m_params->estimation_alpha << std::endl;
  stream << "reactivity=" << m_params->reactivity << std::endl;
} /* show() */

NS_END(params, fordyca);
