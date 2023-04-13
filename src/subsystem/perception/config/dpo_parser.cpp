/**
 * \file dpo_parser.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/subsystem/perception/config/dpo_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, config);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void dpo_parser::parse(const ticpp::Element& node) {
  /* DPO perception not used */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element dnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_pheromone.parse(dnode);
  m_rlos.parse(dnode);
  m_config->pheromone =
      *m_pheromone.config_get<cspconfig::xml::pheromone_parser::config_type>();
  m_config->rlos = *m_rlos.config_get<csprlos::config::xml::rlos_parser::config_type>();
} /* parse() */

bool dpo_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }
  ER_CHECK(m_pheromone.validate(), "Pheromone validation failed");
  ER_CHECK(m_rlos.validate(), "RLOS validation failed");

  return true;

error:
  return false;
} /* validate() */

NS_END(config, perception, subsystem, fordyca);
