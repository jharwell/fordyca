/**
 * @file block_manifest_parser.cpp
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
#include "fordyca/config/arena/block_manifest_parser.hpp"

#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, arena);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_manifest_parser::parse(const ticpp::Element& node) {
  ticpp::Element bnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR_DFLT(bnode, m_config, n_cube, 0U);
  XML_PARSE_ATTR_DFLT(bnode, m_config, n_ramp, 0U);
  XML_PARSE_ATTR(bnode, m_config, unit_dim);
} /* parse() */

bool block_manifest_parser::validate(void) const {
  RCSW_CHECK(m_config->unit_dim > 0);
  RCSW_CHECK(m_config->n_cube > 0 || m_config->n_ramp > 0);
  return true;

error:
  return false;
} /* validate() */

NS_END(arena, config, fordyca);
