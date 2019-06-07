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
#include "fordyca/config/cache_sel/cache_sel_matrix_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, cache_sel);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char cache_sel_matrix_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_sel_matrix_parser::parse(const ticpp::Element& node) {
  ticpp::Element cnode = node_get(node, kXMLRoot);

  m_pickup_policy.parse(cnode);
  m_config =
      std::make_shared<std::remove_reference<decltype(*m_config)>::type>();

  m_config->pickup_policy = *m_pickup_policy.config_get();
  XML_PARSE_ATTR(cnode, m_config, cache_prox_dist);
  XML_PARSE_ATTR(cnode, m_config, block_prox_dist);
  XML_PARSE_ATTR(cnode, m_config, nest_prox_dist);
  XML_PARSE_ATTR(cnode, m_config, cluster_prox_dist);
  XML_PARSE_ATTR(cnode, m_config, site_xrange);
  XML_PARSE_ATTR(cnode, m_config, site_yrange);
} /* parse() */

__rcsw_pure bool cache_sel_matrix_parser::validate(void) const {
  CHECK(m_pickup_policy.validate());
  CHECK(m_config->cache_prox_dist > 0.0);
  CHECK(m_config->block_prox_dist > 0.0);
  CHECK(m_config->nest_prox_dist > 0.0);
  CHECK(m_config->cluster_prox_dist > 0.0);
  return true;

error:
  return false;
} /* validate() */

NS_END(cache_sel, config, fordyca);
