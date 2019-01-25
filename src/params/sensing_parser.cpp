/**
 * @file sensing_parser.cpp
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
#include "fordyca/params/sensing_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char sensing_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void sensing_parser::parse(const ticpp::Element& node) {
  ticpp::Element snode = get_node(const_cast<ticpp::Element&>(node), kXMLRoot);

  m_params =
      std::make_shared<std::remove_reference<decltype(*m_params)>::type>();
  XML_PARSE_ATTR(snode, m_params, los_dim);
  m_proximity_parser.parse(snode);
  m_params->proximity = *m_proximity_parser.parse_results();
} /* parse() */

void sensing_parser::show(std::ostream& stream) const {
  stream << build_header() << XML_ATTR_STR(m_params, los_dim) << std::endl
         << m_proximity_parser << build_footer();
} /* show() */

__rcsw_pure bool sensing_parser::validate(void) const {
  CHECK(m_params->los_dim > 0.0);
  CHECK(m_proximity_parser.validate());
  return true;

error:
  return false;
} /* validate() */

NS_END(params, fordyca);
