/**
 * \file cache_sel_matrix_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/controller/config/cache_sel/cache_sel_matrix_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, cache_sel);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_sel_matrix_parser::parse(const ticpp::Element& node) {
  /* block selection matrix not used */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  m_config = std::make_unique<config_type>();
  ticpp::Element cnode = node_get(node, kXMLRoot);

  m_pickup_policy.parse(cnode);
  m_config->pickup_policy =
      *m_pickup_policy.config_get<cache_pickup_policy_parser::config_type>();

  XML_PARSE_ATTR(cnode, m_config, cache_prox_dist);
  XML_PARSE_ATTR(cnode, m_config, block_prox_dist);
  XML_PARSE_ATTR(cnode, m_config, nest_prox_dist);
  XML_PARSE_ATTR(cnode, m_config, site_xrange);
  XML_PARSE_ATTR(cnode, m_config, site_yrange);
  XML_PARSE_ATTR_DFLT(cnode, m_config, strict_constraints, true);
  XML_PARSE_ATTR(cnode, m_config, new_cache_tol);
} /* parse() */

bool cache_sel_matrix_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }
  ER_CHECK(m_pickup_policy.validate(), "Pickup policy validation failed");
  ER_CHECK(m_config->cache_prox_dist > 0.0,
           "Cache proximity distance must be > 0");
  ER_CHECK(m_config->block_prox_dist > 0.0,
           "Block proximity distance must be > 0");
  ER_CHECK(m_config->nest_prox_dist > 0.0, "Nest proximity distance must be > 0");
  ER_CHECK(m_config->new_cache_tol > 0.0,
           "New cache proximity distance must be > 0");
  return true;

error:
  return false;
} /* validate() */

NS_END(cache_sel, config, controller, fordyca);
