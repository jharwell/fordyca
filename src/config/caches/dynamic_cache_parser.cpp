/**
 * @file dynamic_cache_parser.cpp
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
#include "fordyca/config/caches/dynamic_cache_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, caches);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char dynamic_cache_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void dynamic_cache_parser::parse(const ticpp::Element& node) {
  ticpp::Element cnode = node_get(node, kXMLRoot);
  m_config =
      std::make_shared<std::remove_reference<decltype(*m_config)>::type>();

  XML_PARSE_ATTR(cnode, m_config, enable);
  if (m_config->enable) {
    XML_PARSE_ATTR(cnode, m_config, min_dist);
    XML_PARSE_ATTR(cnode, m_config, min_blocks);
    XML_PARSE_ATTR(cnode, m_config, robot_drop_only);
  }
} /* parse() */

void dynamic_cache_parser::show(std::ostream& stream) const {
  stream << build_header() << std::endl
         << XML_ATTR_STR(m_config, enable) << std::endl
         << XML_ATTR_STR(m_config, min_dist) << std::endl
         << XML_ATTR_STR(m_config, min_blocks) << std::endl
         << XML_ATTR_STR(m_config, robot_drop_only) << std::endl
         << build_footer();
} /* show() */

__rcsw_const bool dynamic_cache_parser::validate(void) const {
  CHECK(m_config->min_dist > 0);
  CHECK(m_config->min_blocks > 0);

error:
  return false;
} /* validate() */

NS_END(caches, config, fordyca);