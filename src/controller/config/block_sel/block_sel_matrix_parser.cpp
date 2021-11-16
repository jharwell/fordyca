/**
 * \file block_sel_matrix_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/controller/config/block_sel/block_sel_matrix_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, config, block_sel);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_sel_matrix_parser::parse(const ticpp::Element& node) {
  /* block selection matrix not used */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }
  ER_DEBUG("Parent node=%s: search for child=%s",
           node.Value().c_str(),
           kXMLRoot.c_str());

  ticpp::Element cnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_priorities.parse(cnode);
  if (m_priorities.is_parsed()) {
    m_config->priorities =
        *m_priorities.config_get<block_priorities_parser::config_type>();
  }

  m_pickup_policy.parse(cnode);
  if (m_pickup_policy.is_parsed()) {
    m_config->pickup_policy =
        *m_pickup_policy.config_get<block_pickup_policy_parser::config_type>();
  }
} /* parse() */

bool block_sel_matrix_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }
  ER_CHECK(m_priorities.validate(), "Priority validation failed");
  ER_CHECK(m_pickup_policy.validate(), "Pickup policy validation failed");

  return true;

error:
  return false;
} /* validate() */

NS_END(block_sel, config, controller, fordyca);
