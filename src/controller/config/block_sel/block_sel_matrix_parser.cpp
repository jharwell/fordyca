/**
 * \file block_sel_matrix_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/config/block_sel/block_sel_matrix_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, block_sel);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_sel_matrix_parser::parse(const ticpp::Element& node) {
  /* block selection matrix not used */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element cnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_priorities.parse(cnode);
  if (m_priorities.is_parsed()) {
    m_config->priorities =
        *m_priorities.config_get<block_priorities_parser::config_type>();
  }

  m_pickup_policy.parse(cnode);
  if (m_pickup_policy.is_parsed()) {
    m_config->pickup_policy =
        *m_pickup_policy.config_get<block_pickup_policy_parser::config_type>();
  }
} /* parse() */

bool block_sel_matrix_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }
  ER_CHECK(m_priorities.validate(), "Priority validation failed");
  ER_CHECK(m_pickup_policy.validate(), "Pickup policy validation failed");

  return true;

error:
  return false;
} /* validate() */

NS_END(block_sel, config, controller, fordyca);
