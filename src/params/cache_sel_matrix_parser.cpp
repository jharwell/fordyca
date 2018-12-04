/**
 * @file cache_sel_matrix_parser.cpp
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
#include "fordyca/params/cache_sel_matrix_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char cache_sel_matrix_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_sel_matrix_parser::parse(const ticpp::Element& node) {
  ticpp::Element cnode = get_node(const_cast<ticpp::Element&>(node), kXMLRoot);
  m_params =
      std::make_shared<std::remove_reference<decltype(*m_params)>::type>();
  XML_PARSE_ATTR(cnode, m_params, cache_prox_dist);
  XML_PARSE_ATTR(cnode, m_params, block_prox_dist);
  XML_PARSE_ATTR(cnode, m_params, nest_prox_dist);
  XML_PARSE_ATTR(cnode, m_params, cluster_prox_dist);
  XML_PARSE_ATTR(cnode, m_params, site_xrange);
  XML_PARSE_ATTR(cnode, m_params, site_yrange);
} /* parse() */

void cache_sel_matrix_parser::show(std::ostream& stream) const {
  stream << build_header() << XML_ATTR_STR(m_params, cache_prox_dist)
         << std::endl
         << XML_ATTR_STR(m_params, cache_prox_dist) << std::endl
         << XML_ATTR_STR(m_params, block_prox_dist) << std::endl
         << XML_ATTR_STR(m_params, nest_prox_dist) << std::endl
         << XML_ATTR_STR(m_params, cluster_prox_dist) << std::endl
         << XML_ATTR_STR(m_params, site_xrange) << std::endl
         << XML_ATTR_STR(m_params, site_yrange) << std::endl
         << build_footer();
} /* show() */

__rcsw_pure bool cache_sel_matrix_parser::validate(void) const {
  CHECK(m_params->cache_prox_dist > 0.0);
  CHECK(m_params->block_prox_dist > 0.0);
  CHECK(m_params->nest_prox_dist > 0.0);
  CHECK(m_params->cluster_prox_dist > 0.0);
  return true;

error:
  return false;
} /* validate() */

NS_END(params, fordyca);
