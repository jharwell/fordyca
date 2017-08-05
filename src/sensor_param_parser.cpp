/**
 * @file sensor_param_parser.cpp
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
#include "fordyca/sensor_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void sensor_param_parser::parse(argos::TConfigurationNode& node) {
  argos::TConfigurationNode diff_node = argos::GetNode(node, "diffusion");

  m_params.reset(new sensor_params);

    try {
      argos::CRange<argos::CDegrees> angle_range_deg(argos::CDegrees(-10.0f),
                                                     argos::CDegrees(10.0f));
      argos::GetNodeAttribute(diff_node,
                              "go_straight_angle_range",
                              m_params->diffusion.go_straight_angle_range);
      m_params->diffusion.go_straight_angle_range.Set(
          argos::ToRadians(angle_range_deg.GetMin()),
          argos::ToRadians(angle_range_deg.GetMax()));
      argos::GetNodeAttribute(diff_node, "delta", m_params->diffusion.delta);
    }
    catch (argos::CARGoSException& ex) {
      using namespace argos;
      THROW_ARGOSEXCEPTION_NESTED("Error parsing sensor parameters.", ex);
    }
} /* parse() */

void sensor_param_parser::show(std::ostream& stream) {
  stream << "Sensor params\n====================" << std::endl;
  stream << "delta=" << m_params->diffusion.delta << std::endl;
  stream << "go_straight_angle_range=" << m_params->diffusion.go_straight_angle_range << std::endl;
} /* show() */

NS_END(fordyca);
