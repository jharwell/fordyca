/**
 * @file logging_parser.cpp
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
#include "fordyca/params/logging_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void logging_parser::parse(argos::TConfigurationNode& node) {
  m_params.reset(new struct logging_params);
  argos::TConfigurationNode lnode = argos::GetNode(node, "logging");
  argos::GetNodeAttribute(lnode, "sim_stats", m_params->sim_stats);
} /* parse() */

void logging_parser::show(std::ostream& stream) {
  stream << "Logging params\n====================" << std::endl;
  stream << "sim_stats=" << m_params->sim_stats << std::endl;
} /* show() */

NS_END(params, fordyca);
