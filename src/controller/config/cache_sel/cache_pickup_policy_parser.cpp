/**
 * \file cache_pickup_policy_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/controller/config/cache_sel/cache_pickup_policy_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, cache_sel);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_pickup_policy_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: search for child=%s",
             node.Value().c_str(),
             kXMLRoot.c_str());

    ticpp::Element cnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();
    XML_PARSE_ATTR(cnode, m_config, policy);
    XML_PARSE_ATTR_DFLT(cnode, m_config, timestep, rtypes::timestep(0));
    XML_PARSE_ATTR_DFLT(cnode, m_config, cache_count, 0UL);
    XML_PARSE_ATTR_DFLT(cnode, m_config, cache_size, 0UL);
  }
} /* parse() */

NS_END(cache_sel, config, controller, fordyca);
