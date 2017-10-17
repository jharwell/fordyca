/**
 * @file arena_map_parser.cpp
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
#include "fordyca/params/arena_map_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void arena_map_parser::parse(argos::TConfigurationNode& node) {
  m_params.reset(new struct arena_map_params);
  m_grid_parser.parse(node);
  m_block_parser.parse(node);
  m_cache_parser.parse(node);
  m_params->grid = *m_grid_parser.get_results();
  m_params->block = *m_block_parser.get_results();
  m_params->cache = *m_cache_parser.get_results();
} /* parse() */

void arena_map_parser::show(std::ostream& stream) {
  stream << "====================\nArena_Map params\n====================\n";
  m_grid_parser.show(stream);
  m_block_parser.show(stream);
  m_cache_parser.show(stream);
} /* show() */

NS_END(params, fordyca);
