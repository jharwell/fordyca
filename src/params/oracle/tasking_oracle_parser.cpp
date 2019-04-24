/**
 * @file tasking_oracle_parser.cpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/params/oracle/tasking_oracle_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, oracle);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char tasking_oracle_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void tasking_oracle_parser::parse(const ticpp::Element& node) {
  ticpp::Element tonode = node_get(node, kXMLRoot);
  m_params =
      std::make_shared<std::remove_reference<decltype(*m_params)>::type>();
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    if (m_params->enabled) {
      XML_PARSE_ATTR(tonode, m_params, task_exec_ests);
      XML_PARSE_ATTR(tonode, m_params, task_interface_ests);
    }
  }
} /* parse() */

NS_END(oracle, params, fordyca);
