/**
 * @file block_priorities_parser.cpp
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
#include "fordyca/config/block_sel/block_priorities_parser.hpp"

#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, block_sel);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_priorities_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ticpp::Element bnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR_DFLT(bnode, m_config, cube, 1.0);
    XML_PARSE_ATTR_DFLT(bnode, m_config, ramp, 1.0);
  }
} /* parse() */

bool block_priorities_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }

  RCSW_CHECK(m_config->cube >= 1.0);
  RCSW_CHECK(m_config->ramp >= 1.0);
  return true;

error:
  return false;
} /* validate() */

NS_END(block_sel, config, fordyca);
