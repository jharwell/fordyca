/**
 * @file loop_functions_parser.cpp
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
#include "fordyca/params/loop_functions_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void loop_functions_parser::parse(argos::TConfigurationNode &node) {
  m_params.reset(new struct loop_functions_params);

  argos::GetNodeAttribute(argos::GetNode(node, "visualization"),
                          "robot_id",
                          m_params->display_robot_id);
  argos::GetNodeAttribute(argos::GetNode(node, "visualization"),
                          "robot_los",
                          m_params->display_robot_los);
  argos::GetNodeAttribute(argos::GetNode(node, "visualization"),
                          "block_id",
                          m_params->display_block_id);
  argos::GetNodeAttribute(argos::GetNode(node, "simulation"),
                          "experiment",
                          m_params->simulation_type);
} /* parse() */

void loop_functions_parser::show(std::ostream &stream) {
  stream << "====================\nLoop Function "
            "params\n====================\n";
  stream << "display_robot_id=" << m_params->display_robot_id << std::endl;
  stream << "display_robot_los=" << m_params->display_robot_los << std::endl;
  stream << "display_block_id=" << m_params->display_block_id << std::endl;
  stream << "simulation_type=" << m_params->simulation_type << std::endl;
} /* show() */

NS_END(params, fordyca);
