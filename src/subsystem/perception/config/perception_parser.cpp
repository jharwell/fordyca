/**
 * \file perception_parser.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/subsystem/perception/config/perception_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, config);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void perception_parser::parse(const ticpp::Element& node) {
  /* Not all robots use a perception subsystem */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element pnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(pnode, m_config, type);

  m_dpo.parse(pnode);
  m_mdpo.parse(pnode);

  if (m_dpo.is_parsed()) {
    m_config->dpo = *m_dpo.config_get<dpo_parser::config_type>();
  }
  if (m_mdpo.is_parsed()) {
    m_config->mdpo = *m_mdpo.config_get<mdpo_parser::config_type>();
  }
} /* parse() */

bool perception_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }
  ER_CHECK(m_dpo.validate(), "DPO validation failed");
  ER_CHECK(m_mdpo.validate(), "MDPO validation failed");

  return true;

error:
  return false;
} /* validate() */

NS_END(config, perception, subsystem, fordyca);
