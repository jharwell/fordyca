/**
 * \file cache_pickup_policy_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
    ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

    ticpp::Element cnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();
    XML_PARSE_ATTR(cnode, m_config, policy);
    XML_PARSE_ATTR_DFLT(cnode, m_config, timestep, rtypes::timestep(0));
    XML_PARSE_ATTR_DFLT(cnode, m_config, cache_count, 0UL);
    XML_PARSE_ATTR_DFLT(cnode, m_config, cache_size, 0UL);
  }
} /* parse() */

NS_END(cache_sel, config, controller, fordyca);
