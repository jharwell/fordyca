/**
 * @file perception_parser.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/config/perception/perception_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, perception);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void perception_parser::parse(const ticpp::Element& node) {
  /*
   * Not all controllers use a perception subsystem
   */
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ticpp::Element onode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    m_occupancy.parse(onode);
    m_pheromone.parse(onode);
    m_config->occupancy_grid =
        *m_occupancy.config_get<grid_parser::config_type>();
    m_config->pheromone =
        *m_pheromone.config_get<pheromone_parser::config_type>();
  }
} /* parse() */

bool perception_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }
  return m_occupancy.validate() && m_pheromone.validate();
} /* validate() */

NS_END(perception, config, fordyca);
