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
#include "fordyca/params/arena/arena_map_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, arena);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char arena_map_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void arena_map_parser::parse(const ticpp::Element& node) {
  ticpp::Element anode = get_node(const_cast<ticpp::Element&>(node), kXMLRoot);
  m_params =
      std::make_shared<std::remove_reference<decltype(*m_params)>::type>();
  m_grid.parse(anode);
  m_params->grid = *m_grid.parse_results();

  m_blocks.parse(anode);
  m_params->blocks = *m_blocks.parse_results();

  m_nest.parse(anode);
  m_params->nest = *m_nest.parse_results();
} /* parse() */

void arena_map_parser::show(std::ostream& stream) const {
  stream << build_header() << m_grid << m_blocks << m_nest
         << build_footer();
} /* show() */

__rcsw_pure bool arena_map_parser::validate(void) const {
  return m_grid.validate() && m_blocks.validate() &&
         m_nest.validate();
} /* validate() */

NS_END(arena, params, fordyca);
