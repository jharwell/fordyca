/**
 * @file perceived_grid_parser.cpp
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
#include "fordyca/params/perceived_grid_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void perceived_grid_parser::parse(argos::TConfigurationNode& node) {
  m_params.reset(new struct perceived_grid_params);

  m_grid_parser.parse(node);
  m_params->grid = *m_grid_parser.get_results();
  m_params->pheromone_rho = std::atof(
      argos::GetNode(node, "grid").GetAttribute("pheromone_rho").c_str());
} /* parse() */

void perceived_grid_parser::show(std::ostream& stream) {
  stream << "====================\nPerceived grid params\n====================\n";
  m_grid_parser.show(stream);
  stream << "pheromone_rho=" << m_params->pheromone_rho << std::endl;
} /* show() */

NS_END(params, fordyca);
