/**
 * @file exec_estimates_parser.cpp
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
#include "fordyca/params/depth1/exec_estimates_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth1);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void exec_estimates_parser::parse(argos::TConfigurationNode& node) {
  m_params = rcppsw::make_unique<struct exec_estimates_params>();

  argos::GetNodeAttribute(node, "enabled", m_params->enabled);
  if (m_params->enabled) {
    argos::GetNodeAttribute(node, "generalist_range", m_params->generalist_range);
    argos::GetNodeAttribute(node, "harvester_range", m_params->harvester_range);
    argos::GetNodeAttribute(node, "collector_range", m_params->collector_range);
  }
} /* parse() */

void exec_estimates_parser::show(std::ostream& stream) {
  stream
      << "====================\nExec estimates params\n====================\n";
  stream << "enabled=" << m_params->enabled << std::endl;
  stream << "generalist_range=" << m_params->generalist_range << std::endl;
  stream << "harvester_range=" << m_params->harvester_range << std::endl;
  stream << "collector_range=" << m_params->collector_range << std::endl;
} /* show() */

NS_END(depth1, params, fordyca);
