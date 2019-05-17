/**
 * @file visualization_parser.cpp
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
#include "fordyca/config/visualization_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char visualization_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void visualization_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ticpp::Element vnode = node_get(node, kXMLRoot);
    m_config =
        std::make_shared<std::remove_reference<decltype(*m_config)>::type>();
    XML_PARSE_ATTR(vnode, m_config, robot_id);
    XML_PARSE_ATTR(vnode, m_config, robot_los);
    XML_PARSE_ATTR(vnode, m_config, robot_task);
    XML_PARSE_ATTR(vnode, m_config, block_id);
    m_parsed = true;
  }
} /* parse() */

void visualization_parser::show(std::ostream& stream) const {
  if (!m_parsed) {
    stream << build_header() << "<< Not parsed >>" << std::endl
           << build_footer();
    return;
  }
  stream << build_header() << XML_ATTR_STR(m_config, robot_id) << std::endl
         << XML_ATTR_STR(m_config, robot_los) << std::endl
         << XML_ATTR_STR(m_config, robot_task) << std::endl
         << XML_ATTR_STR(m_config, block_id) << std::endl
         << build_footer();
} /* show() */

NS_END(config, fordyca);
