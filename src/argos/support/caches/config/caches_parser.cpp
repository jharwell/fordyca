/**
 * \file caches_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/argos/support/caches/config/caches_parser.hpp"

#include "rcppsw/math/math.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, caches, config);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void caches_parser::parse(const ticpp::Element& node) {
  /* caches not used */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element cnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_static.parse(cnode);
  if (m_static.is_parsed()) {
    m_config->static_ = *m_static.config_get<static_cache_parser::config_type>();
  }

  m_dynamic.parse(cnode);
  if (m_dynamic.is_parsed()) {
    m_config->dynamic =
        *m_dynamic.config_get<dynamic_cache_parser::config_type>();
  }

  XML_PARSE_ATTR(cnode, m_config, dimension);
  m_config->dimension += rspatial::euclidean_dist(rmath::kDOUBLE_EPSILON);
  XML_PARSE_ATTR_DFLT(cnode, m_config, strict_constraints, true);
} /* parse() */

bool caches_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }

  ER_CHECK(m_config->dimension > 0.0, "Dimension must be > 0");
  ER_CHECK(m_dynamic.validate(), "Dynamic validation failed");
  ER_CHECK(m_static.validate(), "Static validation failed");
  return true;

error:
  return false;
} /* validate() */

NS_END(config, caches, support, argos, fordyca);
