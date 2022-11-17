/**
 * \file block_priorities_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/config/block_sel/block_priorities_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, block_sel);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_priorities_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

    ticpp::Element bnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR_DFLT(bnode, m_config, cube, 1.0);
    XML_PARSE_ATTR_DFLT(bnode, m_config, ramp, 1.0);
  }
} /* parse() */

bool block_priorities_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }

  ER_CHECK(m_config->cube >= 1.0, "Cube priority must be >= 1.0");
  ER_CHECK(m_config->ramp >= 1.0, "Ramp priority must be >= 1.0");
  return true;

error:
  return false;
} /* validate() */

NS_END(block_sel, config, controller, fordyca);
