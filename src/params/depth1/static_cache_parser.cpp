/**
 * @file static_cache_parser.cpp
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
#include <argos3/core/utility/configuration/argos_configuration.h>

#include "fordyca/params/depth1/static_cache_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth1);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char static_cache_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void static_cache_parser::parse(const ticpp::Element& node) {
  ticpp::Element bnode =
      argos::GetNode(const_cast<ticpp::Element&>(node), kXMLRoot);
  XML_PARSE_PARAM(bnode, m_params, enable);
  if (m_params.enable) {
    XML_PARSE_PARAM(bnode, m_params, size);
    XML_PARSE_PARAM(bnode, m_params, dimension);
    XML_PARSE_PARAM(bnode, m_params, min_dist);
    XML_PARSE_PARAM(bnode, m_params, usage_penalty);
    XML_PARSE_PARAM(bnode, m_params, respawn_scale_factor);
  }
} /* parse() */

void static_cache_parser::show(std::ostream& stream) const {
  stream << build_header() << std::endl
         << XML_PARAM_STR(m_params, enable) << std::endl
         << XML_PARAM_STR(m_params, size) << std::endl
         << XML_PARAM_STR(m_params, dimension) << std::endl
         << XML_PARAM_STR(m_params, min_dist) << std::endl
         << XML_PARAM_STR(m_params, usage_penalty) << std::endl
         << XML_PARAM_STR(m_params, respawn_scale_factor) << std::endl
         << build_footer();
} /* show() */

__pure bool static_cache_parser::validate(void) const {
  if (m_params.enable) {
    CHECK(m_params.dimension > 0.0);
    CHECK(m_params.size > 0.0);
    CHECK(m_params.respawn_scale_factor > 0.0);
    return true;
  }

error:
  return false;
} /* validate() */

NS_END(depth1, params, fordyca);
