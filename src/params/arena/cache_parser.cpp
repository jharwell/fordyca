/**
 * @file cache_parser.cpp
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
#include "fordyca/params/arena/cache_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, arena);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char cache_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ticpp::Element cnode = get_node(const_cast<ticpp::Element&>(node), kXMLRoot);
    m_params =
        std::make_shared<std::remove_reference<decltype(*m_params)>::type>();
    XML_PARSE_ATTR(cnode, m_params, static_size);
    XML_PARSE_ATTR(cnode, m_params, dimension);
    XML_PARSE_ATTR(cnode, m_params, min_dist);
    XML_PARSE_ATTR(cnode, m_params, respawn_scale_factor);
    m_waveform.parse(
        get_node(const_cast<ticpp::Element&>(cnode), "usage_penalty"));
    m_params->usage_penalty = *m_waveform.parse_results();

    m_parsed = true;
  }
} /* parse() */

void cache_parser::show(std::ostream& stream) const {
  if (!m_parsed) {
    stream << build_header() << "<< Not Parsed >>" << std::endl
           << build_footer();
    return;
  }

  stream << build_header() << std::endl
         << XML_ATTR_STR(m_params, static_size) << std::endl
         << XML_ATTR_STR(m_params, dimension) << std::endl
         << XML_ATTR_STR(m_params, min_dist) << std::endl
         << XML_ATTR_STR(m_params, respawn_scale_factor) << std::endl
         << m_waveform << build_footer();
} /* show() */

__rcsw_pure bool cache_parser::validate(void) const {
  if (m_parsed) {
    CHECK(m_params->dimension > 0.0);
    CHECK(m_params->static_size > 0);
    CHECK(m_params->respawn_scale_factor > 0.0);
    CHECK(true == m_waveform.validate());
    return true;
  }

error:
  return false;
} /* validate() */

NS_END(arena, params, fordyca);
