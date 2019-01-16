/**
 * @file pheromone_parser.cpp
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
#include "fordyca/params/perception/pheromone_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, perception);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char pheromone_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void pheromone_parser::parse(const ticpp::Element& node) {
  ticpp::Element pnode = get_node(const_cast<ticpp::Element&>(node), kXMLRoot);
  m_params =
      std::make_shared<std::remove_reference<decltype(*m_params)>::type>();
  XML_PARSE_ATTR(pnode, m_params, rho);
  XML_PARSE_ATTR(pnode, m_params, repeat_deposit);
} /* parse() */

__rcsw_pure bool pheromone_parser::validate(void) const {
  return m_params->rho > 0.0;
} /* validate() */

NS_END(perception, params, fordyca);