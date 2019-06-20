/**
 * @file arena_map_parser.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/config/arena/arena_map_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, arena);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char arena_map_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void arena_map_parser::parse(const ticpp::Element& node) {
  ticpp::Element anode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_grid.parse(anode);
  m_config->grid = *m_grid.config_get<grid_parser::config_type>();

  m_blocks.parse(anode);
  m_config->blocks = *m_blocks.config_get<blocks_parser::config_type>();

  m_nest.parse(anode);
  m_config->nest = *m_nest.config_get<nest_parser::config_type>();
} /* parse() */

__rcsw_pure bool arena_map_parser::validate(void) const {
  return m_grid.validate() && m_blocks.validate() && m_nest.validate();
} /* validate() */

NS_END(arena, config, fordyca);
