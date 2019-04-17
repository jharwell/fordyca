/**
 * @file proximity_sensor_parser.cpp
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
#include "fordyca/params/proximity_sensor_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char proximity_sensor_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void proximity_sensor_parser::parse(const ticpp::Element& node) {
  ticpp::Element pnode = node_get(node, kXMLRoot);
  m_params =
      std::make_shared<std::remove_reference<decltype(*m_params)>::type>();
  XML_PARSE_ATTR(pnode, m_params, delta);
} /* parse() */

void proximity_sensor_parser::show(std::ostream& stream) const {
  stream << build_header() << XML_ATTR_STR(m_params, delta) << std::endl
         << build_footer();
} /* show() */

__rcsw_pure bool proximity_sensor_parser::validate(void) const {
  CHECK(m_params->delta > 0.0);
  return true;

error:
  return false;
} /* validate() */

NS_END(params, fordyca);
