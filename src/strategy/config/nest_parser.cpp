/**
 * \file nest_parser.cpp
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
