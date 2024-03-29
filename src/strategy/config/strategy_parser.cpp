/**
 * \file strategy_parser.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "fordyca/strategy/config/strategy_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, strategy, config);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void strategy_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element snode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_blocks.parse(snode);
  m_caches.parse(snode);
  m_nest.parse(snode);

  if (m_blocks.is_parsed()) {
    m_config->blocks = *m_blocks.config_get<blocks_parser::config_type>();
  }
  if (m_caches.is_parsed()) {
    m_config->caches = *m_caches.config_get<caches_parser::config_type>();
  }
  if (m_nest.is_parsed()) {
    m_config->nest = *m_nest.config_get<nest_parser::config_type>();
  }
} /* parse() */

NS_END(config, strategy, fordyca);
