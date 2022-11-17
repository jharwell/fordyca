/**
 * \file caches_parser.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/strategy/config/caches_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, strategy, config);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void caches_parser::parse(const ticpp::Element& node) {
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element vnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_explore.parse(vnode);

  if (m_explore.is_parsed()) {
    m_config->explore =
        *m_explore.config_get<cssexplore::config::xml::explore_parser::config_type>();
  }
} /* parse() */

bool caches_parser::validate(void) const {
  ER_CHECK(m_explore.validate(), "Cache exploration config validation failed");

  return true;

error:
  return false;
} /* validate() */

NS_END(config, strategy, fordyca);
