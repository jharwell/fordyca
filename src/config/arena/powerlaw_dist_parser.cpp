/**
 * @file powerlaw_dist_parser.cpp
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
#include "fordyca/config/arena/powerlaw_dist_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, arena);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char powerlaw_dist_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void powerlaw_dist_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ticpp::Element bnode = node_get(node, kXMLRoot);
    m_config =
        std::make_shared<std::remove_reference<decltype(*m_config)>::type>();
    XML_PARSE_ATTR(bnode, m_config, pwr_min);
    XML_PARSE_ATTR(bnode, m_config, pwr_max);
    XML_PARSE_ATTR(bnode, m_config, n_clusters);
    m_parsed = true;
  }
} /* parse() */

void powerlaw_dist_parser::show(std::ostream& stream) const {
  stream << build_header();
  if (!m_parsed) {
    stream << "<<  Not Parsed >>" << std::endl << build_footer();
    return;
  }
  stream << XML_ATTR_STR(m_config, pwr_min) << std::endl
         << XML_ATTR_STR(m_config, pwr_max) << std::endl
         << XML_ATTR_STR(m_config, n_clusters) << std::endl
         << build_footer();
} /* show() */

__rcsw_pure bool powerlaw_dist_parser::validate(void) const {
  CHECK(m_config->pwr_min > 2);
  CHECK(m_config->pwr_max >= m_config->pwr_min);
  CHECK(m_config->n_clusters > 0);
  return true;

error:
  return false;
} /* validate() */

NS_END(arena, config, fordyca);