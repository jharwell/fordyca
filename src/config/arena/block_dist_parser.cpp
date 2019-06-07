/**
 * @file block_dist_parser.cpp
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
#include "fordyca/config/arena/block_dist_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, arena);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char block_dist_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_dist_parser::parse(const ticpp::Element& node) {
  ticpp::Element bnode = node_get(node, kXMLRoot);
  m_powerlaw.parse(bnode);
  m_manifest.parse(bnode);
  m_redist_governor.parse(bnode);
  m_config =
      std::make_shared<std::remove_reference<decltype(*m_config)>::type>();
  XML_PARSE_ATTR(bnode, m_config, arena_resolution);
  XML_PARSE_ATTR(bnode, m_config, dist_type);
  m_config->powerlaw = *m_powerlaw.config_get();
  m_config->manifest = *m_manifest.config_get();
  m_config->redist_governor = *m_redist_governor.config_get();
} /* parse() */

__rcsw_pure bool block_dist_parser::validate(void) const {
  CHECK(true == m_powerlaw.validate());
  CHECK(true == m_manifest.validate());
  CHECK(true == m_redist_governor.validate());
  return true;

error:
  return false;
} /* validate() */

NS_END(arena, config, fordyca);
