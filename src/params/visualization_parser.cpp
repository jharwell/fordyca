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
#include <argos3/core/utility/configuration/argos_configuration.h>

#include "fordyca/params/visualization_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char visualization_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void visualization_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ticpp::Element vnode = argos::GetNode(const_cast<ticpp::Element&>(node),
                                          kXMLRoot);
    XML_PARSE_PARAM(vnode, m_params, robot_id);
    XML_PARSE_PARAM(vnode, m_params, robot_los);
    XML_PARSE_PARAM(vnode, m_params, robot_task);
    XML_PARSE_PARAM(vnode, m_params, block_id);
    m_parsed = true;
  }
} /* parse() */

void visualization_parser::show(std::ostream& stream) const {
  if (!m_parsed) {
    stream << "<< Not parsed >>" << std::endl << build_footer();
    return;
  }
  stream << build_header()
         << XML_PARAM_STR(m_params, robot_id) << std::endl
         << XML_PARAM_STR(m_params, robot_los) << std::endl
         << XML_PARAM_STR(m_params, robot_task) << std::endl
         << XML_PARAM_STR(m_params, block_id) << std::endl
         << build_footer();
} /* show() */

NS_END(params, fordyca);
