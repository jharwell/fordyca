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
#include "fordyca/params/arena/block_manifest_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, arena);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char block_manifest_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_manifest_parser::parse(const ticpp::Element& node) {
  ticpp::Element bnode = node_get(node, kXMLRoot);
  m_params =
      std::make_shared<std::remove_reference<decltype(*m_params)>::type>();
  XML_PARSE_ATTR(bnode, m_params, n_cube);
  XML_PARSE_ATTR(bnode, m_params, n_ramp);
  XML_PARSE_ATTR(bnode, m_params, unit_dim);
} /* parse() */

void block_manifest_parser::show(std::ostream& stream) const {
  stream << build_header() << XML_ATTR_STR(m_params, n_cube) << std::endl
         << XML_ATTR_STR(m_params, n_ramp) << std::endl
         << XML_ATTR_STR(m_params, unit_dim) << std::endl
         << build_footer();
} /* show() */

__rcsw_pure bool block_manifest_parser::validate(void) const {
  CHECK(m_params->unit_dim > 0);
  return true;

error:
  return false;
} /* validate() */

NS_END(arena, params, fordyca);
