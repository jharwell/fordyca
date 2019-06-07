/**
 * @file pickup_policy_parser.cpp
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
#include "fordyca/config/cache_sel/pickup_policy_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, cache_sel);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char pickup_policy_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void pickup_policy_parser::parse(const ticpp::Element& node) {
  /*
   * Needs to be populated always so we get the null trigger when the policy is
   * disabled.
   */
  m_config =
      std::make_shared<std::remove_reference<decltype(*m_config)>::type>();
  m_config->policy = "Null";

  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ticpp::Element cnode = node_get(node, kXMLRoot);
    XML_PARSE_ATTR(cnode, m_config, policy);
    XML_PARSE_ATTR_DFLT(cnode, m_config, timestep, 0U);
    XML_PARSE_ATTR_DFLT(cnode, m_config, cache_count, 0U);
    XML_PARSE_ATTR_DFLT(cnode, m_config, cache_size, 0U);
  }
} /* parse() */

NS_END(cache_sel, config, fordyca);
