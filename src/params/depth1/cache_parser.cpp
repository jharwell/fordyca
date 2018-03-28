/**
 * @file cache_parser.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#include "fordyca/params/depth1/cache_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth1);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char cache_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_parser::parse(const ticpp::Element& node) {
  ticpp::Element bnode =
      argos::GetNode(const_cast<ticpp::Element&>(node), kXMLRoot);
  XML_PARSE_PARAM(bnode, m_params, dimension);
  XML_PARSE_PARAM(bnode, m_params, min_dist);
  XML_PARSE_PARAM(bnode, m_params, static_size);
  XML_PARSE_PARAM(bnode, m_params, usage_penalty);
  XML_PARSE_PARAM(bnode, m_params, create_static);
  XML_PARSE_PARAM(bnode, m_params, create_dynamic);
  XML_PARSE_PARAM(bnode, m_params, static_respawn_scale_factor);
} /* parse() */

void cache_parser::show(std::ostream& stream) const {
  stream << build_header() << std::endl
         << XML_PARAM_STR(m_params, dimension) << std::endl
         << XML_PARAM_STR(m_params, min_dist) << std::endl
         << XML_PARAM_STR(m_params, static_size) << std::endl
         << XML_PARAM_STR(m_params, usage_penalty) << std::endl
         << XML_PARAM_STR(m_params, create_static) << std::endl
         << XML_PARAM_STR(m_params, create_dynamic) << std::endl
         << XML_PARAM_STR(m_params, static_respawn_scale_factor) << std::endl;
} /* show() */

__pure bool cache_parser::validate(void) const {
  if (m_params.dimension <= 0.0) {
    return false;
  }
  if (m_params.create_static && 0 == m_params.static_size) {
    return false;
  }
  if (m_params.static_respawn_scale_factor <= 0.0) {
    return false;
  }
  return true;
} /* validate() */

NS_END(depth1, params, fordyca);
