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
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char arena_map_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void arena_map_parser::parse(const ticpp::Element& node) {
  ticpp::Element anode =
      argos::GetNode(const_cast<ticpp::Element&>(node), kXMLRoot);
  m_params =
      std::make_shared<std::remove_reference<decltype(*m_params)>::type>();
  m_grid_parser.parse(anode);
  m_params->grid = *m_grid_parser.parse_results();

  m_block_parser.parse(anode);
  m_params->block = *m_block_parser.parse_results();

  m_block_dist_parser.parse(anode);
  m_params->block_dist = *m_block_dist_parser.parse_results();

  m_cache_parser.parse(anode);
  if (m_cache_parser.parsed()) {
    m_params->static_cache = *m_cache_parser.parse_results();
  }

  m_nest_parser.parse(anode);
  m_params->nest = *m_nest_parser.parse_results();
} /* parse() */

void arena_map_parser::show(std::ostream& stream) const {
  stream << build_header() << m_grid_parser << m_block_parser << m_block_dist_parser << m_cache_parser
         << m_nest_parser << build_footer();
} /* show() */

__rcsw_pure bool arena_map_parser::validate(void) const {
  return m_grid_parser.validate() && m_block_parser.validate() &&
      m_block_dist_parser.validate() && m_cache_parser.validate() && m_nest_parser.validate();
} /* validate() */

NS_END(params, fordyca);
