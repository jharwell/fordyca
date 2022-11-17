/**
 * \file nest_parser.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/strategy/config/nest_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, strategy, config);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nest_parser::parse(const ticpp::Element& node) {
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element vnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_acq.parse(vnode);
  m_exit.parse(vnode);

  if (m_acq.is_parsed()) {
    m_config->acq =
        *m_acq.config_get<cssnest::config::xml::acq_parser::config_type>();
  }
  if (m_exit.is_parsed()) {
    m_config->exit =
        *m_exit.config_get<cssnest::config::xml::exit_parser::config_type>();
  }
} /* parse() */

bool nest_parser::validate(void) const {
  ER_CHECK(m_acq.validate(), "Nest acquisition config validation failed");
  ER_CHECK(m_exit.validate(), "Nest exit config validation failed");

  return true;

error:
  return false;
} /* validate() */

NS_END(config, strategy, fordyca);
