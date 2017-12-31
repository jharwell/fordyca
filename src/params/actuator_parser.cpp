/**
 * @file actuator_parser.cpp
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
#include "fordyca/params/actuator_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void actuator_parser::parse(argos::TConfigurationNode &node) {
  argos::TConfigurationNode wheel_node =
      argos::GetNode(argos::GetNode(node, "actuators"), "wheels");

  m_params = rcppsw::make_unique<struct actuator_params>();

    argos::CDegrees angle;
    argos::GetNodeAttribute(wheel_node, "soft_turn_angle_max", angle);
    m_params->wheels.soft_turn_max = ToRadians(angle);
    argos::GetNodeAttribute(wheel_node, "no_turn_angle_max", angle);
    m_params->wheels.no_turn_max = ToRadians(angle);
    argos::GetNodeAttribute(wheel_node,
                            "max_speed",
                            m_params->wheels.max_speed);
} /* parse() */

void actuator_parser::show(std::ostream &stream) {
  stream << "====================\nActuator params\n====================\n";
  stream << "soft_turn_max=" << m_params->wheels.soft_turn_max
         << std::endl;
  stream << "no_turn_max=" << m_params->wheels.no_turn_max
         << std::endl;
  stream << "max_speed=" << m_params->wheels.max_speed << std::endl;
} /* show() */

bool actuator_parser::validate(void) {
  if (!(m_params->wheels.soft_turn_max.GetValue() > 0)) {
    return false;
  }
  if (!(m_params->wheels.no_turn_max.GetValue() > 0)) {
    return false;
  }
  return true;
} /* validate() */

NS_END(params, fordyca);
