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
#include "fordyca/params/depth0/pheromone_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth0);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void pheromone_parser::parse(argos::TConfigurationNode &node) {
  m_params = rcppsw::make_unique<struct pheromone_params>();
  m_params->rho = std::atof(node.GetAttribute("rho").c_str());
  argos::GetNodeAttribute(node, "repeat_deposit", m_params->repeat_deposit);
} /* parse() */

void pheromone_parser::show(std::ostream &stream) {
  stream << "====================\nPheromone params\n====================\n";
  stream << "rho=" << m_params->rho << std::endl;
  stream << "repeat_deposit=" << m_params->repeat_deposit << std::endl;
} /* show() */

__pure bool pheromone_parser::validate(void) {
  return m_params->rho > 0.0;
} /* validate() */

NS_END(depth0, params, fordyca);
