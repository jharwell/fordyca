/**
 * @file entities_oracle_parser.cpp
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
#include "fordyca/config/oracle/entities_oracle_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, oracle);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char entities_oracle_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void entities_oracle_parser::parse(const ticpp::Element& node) {
  /*
   * Needs to be populated always so we it is disabled by default rather than a
   * nullptr and a segfault.
   */
  m_config =
      std::make_shared<std::remove_reference<decltype(*m_config)>::type>();
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ticpp::Element cnode = node_get(node, kXMLRoot);
    XML_PARSE_ATTR(cnode, m_config, caches_enabled);
    XML_PARSE_ATTR(cnode, m_config, blocks_enabled);
  }
} /* parse() */

NS_END(oracle, config, fordyca);
