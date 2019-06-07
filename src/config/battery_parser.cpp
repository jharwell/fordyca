/**
 * @file sensing_parser.cpp
 *
 * @copyright 2018 Nimer WazWaz, All rights reserved.
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
#include "fordyca/config/battery_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char battery_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void battery_parser::parse(const ticpp::Element& node) {
  ticpp::Element bynode = node_get(node, kXMLRoot);

  m_config =
      std::make_shared<std::remove_reference<decltype(*m_config)>::type>();
  XML_PARSE_ATTR(bynode, m_config, power_station_amount);
  XML_PARSE_ATTR(bynode, m_config, power_station_size);
  XML_PARSE_ATTR(bynode, m_config, power_station_distrubtion);
}

__rcsw_pure bool battery_parser::validate(void) const {
  if (m_config->power_station_size < 0) {
    return false;
  } else if (m_config->power_station_amount < 0) {
    return false;
  }
  return true;
} /* validate() */

NS_END(config, fordyca);
