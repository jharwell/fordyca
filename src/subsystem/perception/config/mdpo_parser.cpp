/**
 * \file mdpo_parser.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * This file is part of FORDYCA.
 *
 * FORDYCA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * FORDYCA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * FORDYCA.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/subsystem/perception/config/mdpo_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, config);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void mdpo_parser::parse(const ticpp::Element& node) {
  /* MDPO perception not used */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }
  ER_DEBUG("Parent node=%s: search for child=%s",
           node.Value().c_str(),
           kXMLRoot.c_str());

  ticpp::Element mnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_pheromone.parse(mnode);
  m_rlos.parse(mnode);

  m_config->pheromone = *m_pheromone.config_get<cspconfig::xml::pheromone_parser::config_type>();
  m_config->rlos = *m_rlos.config_get<cspconfig::xml::rlos_parser::config_type>();
} /* parse() */

bool mdpo_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }
  ER_CHECK(m_pheromone.validate(), "Pheromone validation failed");
  ER_CHECK(m_rlos.validate(), "RLOS validationb failed");

  return true;

error:
  return false;
} /* validate() */

NS_END(config, perception, subsystem, fordyca);
