/**
 * @file block_redist_governor_parser.cpp
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
#include "fordyca/config/arena/block_redist_governor_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, arena);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char block_redist_governor_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_redist_governor_parser::parse(const ticpp::Element& node) {
  /*
   * Needs to be populated always so we get the null trigger when the governor
   * is disabled.
   */
  m_config =
      std::make_shared<std::remove_reference<decltype(*m_config)>::type>();

  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ticpp::Element lnode = node_get(node, kXMLRoot);
    XML_PARSE_ATTR_DFLT(lnode, m_config, timestep, 0U);
    XML_PARSE_ATTR_DFLT(lnode, m_config, block_count, 0U);
    XML_PARSE_ATTR(lnode, m_config, trigger);
    XML_PARSE_ATTR(lnode, m_config, recurrence_policy);
  } else {
    m_config->trigger = "Null";
  }
} /* parse() */

NS_END(arena, config, fordyca);
