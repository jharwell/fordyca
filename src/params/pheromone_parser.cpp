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
#include "fordyca/params/pheromone_parser.hpp"
#include <argos3/core/utility/configuration/argos_configuration.h>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char pheromone_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void pheromone_parser::parse(const ticpp::Element& node) {
  ticpp::Element pnode =
      argos::GetNode(const_cast<ticpp::Element&>(node), kXMLRoot);
  m_params =
      std::make_shared<std::remove_reference<decltype(*m_params)>::type>();
  XML_PARSE_PARAM(pnode, m_params, rho);
  XML_PARSE_PARAM(pnode, m_params, repeat_deposit);
} /* parse() */

void pheromone_parser::show(std::ostream& stream) const {
  stream << build_header()
         << XML_PARAM_STR(m_params, rho) << std::endl
         << XML_PARAM_STR(m_params, repeat_deposit) << std::endl
         << build_footer();
} /* show() */

__rcsw_pure bool pheromone_parser::validate(void) const {
  return m_params->rho > 0.0;
} /* validate() */

NS_END(params, fordyca);
