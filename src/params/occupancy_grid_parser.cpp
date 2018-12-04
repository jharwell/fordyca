/**
 * @file occupancy_grid_parser.cpp
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
#include "fordyca/params/occupancy_grid_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char occupancy_grid_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void occupancy_grid_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ticpp::Element onode = get_node(const_cast<ticpp::Element&>(node), kXMLRoot);
    m_params =
        std::make_shared<std::remove_reference<decltype(*m_params)>::type>();

    m_grid.parse(onode);
    m_pheromone.parse(onode);
    m_params->grid = *m_grid.parse_results();
    m_params->pheromone = *m_pheromone.parse_results();
    m_parsed = true;
  }
} /* parse() */

void occupancy_grid_parser::show(std::ostream& stream) const {
  if (!m_parsed) {
    stream << build_header() << "<< Not parsed >>" << std::endl
           << build_footer();
    return;
  }
  stream << build_header() << m_grid << m_pheromone << build_footer();
} /* show() */

__rcsw_pure bool occupancy_grid_parser::validate(void) const {
  if (m_parsed) {
    return m_grid.validate() && m_pheromone.validate();
  }
  return true;
} /* validate() */

NS_END(params, fordyca);
