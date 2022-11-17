/**
 * \file block_pickup_policy_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/config/block_sel/block_pickup_policy_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, block_sel);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_pickup_policy_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

    ticpp::Element cnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR(cnode, m_config, policy);
    XML_PARSE_ATTR_DFLT(cnode, m_config, prox_dist, 0.0);
  }
} /* parse() */

NS_END(block_sel, config, controller, fordyca);
