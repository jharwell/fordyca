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
#include "fordyca/params/arena/block_dist_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, arena);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char block_dist_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_dist_parser::parse(const ticpp::Element& node) {
  ticpp::Element bnode = get_node(const_cast<ticpp::Element&>(node), kXMLRoot);
  m_powerlaw.parse(bnode);
  m_manifest.parse(bnode);
  m_params =
      std::make_shared<std::remove_reference<decltype(*m_params)>::type>();
  XML_PARSE_ATTR(bnode, m_params, arena_resolution);
  XML_PARSE_ATTR(bnode, m_params, dist_type);
  m_params->powerlaw = *m_powerlaw.parse_results();
  m_params->manifest = *m_manifest.parse_results();
} /* parse() */

void block_dist_parser::show(std::ostream& stream) const {
  stream << build_header() << XML_ATTR_STR(m_params, dist_type) << std::endl
         << XML_ATTR_STR(m_params, arena_resolution) << std::endl
         << XML_ATTR_STR(m_params, dist_type) << std::endl
         << m_powerlaw << build_footer();
} /* show() */

bool block_dist_parser::validate(void) const {
  CHECK(true == m_powerlaw.validate());
  CHECK(true == m_manifest.validate());
  CHECK("" != m_params->dist_type);
  return true;

error:
  return false;
} /* validate() */

NS_END(arena, params, fordyca);
