/**
 * @file wheel_parser.cpp
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
#include "fordyca/params/wheel_parser.hpp"
#include <argos3/core/utility/configuration/argos_configuration.h>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char wheel_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void wheel_parser::parse(const ticpp::Element& node) {
  ticpp::Element wnode =
      argos::GetNode(const_cast<ticpp::Element&>(node), kXMLRoot);

  XML_PARSE_PARAM(wnode, m_params, max_speed);
  XML_PARSE_PARAM(wnode, m_params, soft_turn_max);
  XML_PARSE_PARAM(wnode, m_params, no_turn_max);

  argos::CDegrees angle;
  argos::GetNodeAttribute(wnode, "soft_turn_max", angle);
  m_params.soft_turn_max = argos::ToRadians(angle);

  argos::GetNodeAttribute(wnode, "no_turn_max", angle);
  m_params.no_turn_max = argos::ToRadians(angle);
} /* parse() */

void wheel_parser::show(std::ostream& stream) const {
  stream << build_header()
         << XML_PARAM_STR(m_params, soft_turn_max) << std::endl
         << XML_PARAM_STR(m_params, no_turn_max) << std::endl
         << XML_PARAM_STR(m_params, max_speed) << std::endl
         << build_footer();
} /* show() */

__pure bool wheel_parser::validate(void) const {
  if (!(m_params.soft_turn_max.GetValue() > 0)) {
    return false;
  }
  if (!(m_params.no_turn_max.GetValue() > 0)) {
    return false;
  }
  if (!(m_params.no_turn_max < m_params.soft_turn_max)) {
    return false;
  }
  return true;
} /* validate() */

NS_END(params, fordyca);
