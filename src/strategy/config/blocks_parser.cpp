/**
 * \file blocks_parser.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/strategy/config/blocks_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, strategy, config);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void blocks_parser::parse(const ticpp::Element& node) {
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element vnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_drop.parse(vnode);
  m_explore.parse(vnode);

  if (m_drop.is_parsed()) {
    m_config->drop =
        *m_drop.config_get<cssblocks::config::xml::drop_parser::config_type>();
  }
  if (m_explore.is_parsed()) {
    m_config->explore =
        *m_explore.config_get<cssexplore::config::xml::explore_parser::config_type>();
  }
} /* parse() */

bool blocks_parser::validate(void) const {
  ER_CHECK(m_drop.validate(), "Block drop config validation failed");
  ER_CHECK(m_explore.validate(), "Bloc exploration config validation failed");

  return true;

error:
  return false;
} /* validate() */

NS_END(config, strategy, fordyca);
