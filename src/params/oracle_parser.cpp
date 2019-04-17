/**
 * @file oracle_parser.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/params/oracle_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char oracle_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void oracle_parser::parse(const ticpp::Element& node) {
  ticpp::Element enode = node_get(node, kXMLRoot);
  m_params =
      std::make_shared<std::remove_reference<decltype(*m_params)>::type>();
  XML_PARSE_ATTR(enode, m_params, enabled);
  if (m_params->enabled) {
    XML_PARSE_ATTR(enode, m_params, task_exec_ests);
    XML_PARSE_ATTR(enode, m_params, task_interface_ests);
  }
} /* parse() */

void oracle_parser::show(std::ostream& stream) const {
  stream << build_header() << XML_ATTR_STR(m_params, enabled) << std::endl
         << XML_ATTR_STR(m_params, task_exec_ests) << std::endl
         << XML_ATTR_STR(m_params, task_interface_ests) << std::endl
         << build_footer();
} /* show() */

NS_END(params, fordyca);
