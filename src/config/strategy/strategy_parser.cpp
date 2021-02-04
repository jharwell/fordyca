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
#include "fordyca/config/strategy/strategy_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, strategy);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void strategy_parser::parse(const ticpp::Element& node) {
  ticpp::Element snode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_explore.parse(snode);
  m_nest_acq.parse(snode);

  if (m_explore.is_parsed()) {
    m_config->explore = *m_explore.config_get<explore_parser::config_type>();
  }
  if (m_nest_acq.is_parsed()) {
    m_config->nest_acq = *m_nest_acq.config_get<csstrategy::config::xml::nest_acq_parser::config_type>();
  }
} /* parse() */

NS_END(config, fordyca, strategy);
