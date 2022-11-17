/**
 * \file static_cache_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/argos/support/caches/config/static_cache_parser.hpp"

#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, caches, config);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void static_cache_parser::parse(const ticpp::Element& node) {
  /* static caches not used */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element cnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(cnode, m_config, enable);
  if (m_config->enable) {
    XML_PARSE_ATTR(cnode, m_config, size);
    XML_PARSE_ATTR(cnode, m_config, respawn_scale_factor);
  }
} /* parse() */

bool static_cache_parser::validate(void) const {
  if (!is_parsed() || (is_parsed() && !m_config->enable)) {
    return true;
  }

  ER_CHECK(m_config->size > 0, "Size must be > 0");
  ER_CHECK(m_config->respawn_scale_factor > 0, "Scale factor must be > 0");
  return true;

error:
  return false;
} /* validate() */

NS_END(config, caches, support, argos, fordyca);
