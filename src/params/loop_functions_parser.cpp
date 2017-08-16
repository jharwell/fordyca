/**
 * @file loop_functions_parser.cpp
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
#include "fordyca/params/loop_functions_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void loop_functions_parser::parse(argos::TConfigurationNode& node) {
  m_params.reset(new struct loop_functions_params);
  ticpp::Node *arena = node.NextSibling("arena");
  std::vector<std::string> res, res2;

  rcppsw::utils::line_parser parser(' ');
  res = parser.parse(arena->ToElement()->GetAttribute("size"));

  /*
   * +/- 0.3 is for the walls of the arena which reduce the effective size.
   */
  m_params->arena_x.Set(-std::atoi(res[0].c_str())/2.0 + 0.3,
                        std::atoi(res[0].c_str())/2.0 - 0.3);
  m_params->arena_y.Set(-std::atoi(res[1].c_str())/2.0 + 0.3,
                        std::atoi(res[1].c_str())/2.0 - 0.3);
  res = parser.parse(node.FirstChildElement("nest")->GetAttribute("center"));
  res2 = parser.parse(node.FirstChildElement("nest")->GetAttribute("size"));

  m_params->nest_x.Set(std::atof(res[0].c_str()) - std::atof(res2[0].c_str()),
                       std::atof(res[0].c_str()) + std::atof(res2[0].c_str()));
  m_params->nest_y.Set(std::atof(res[1].c_str()) - std::atof(res2[1].c_str()),
                       std::atof(res[1].c_str()) + std::atof(res2[1].c_str()));
} /* parse() */

void loop_functions_parser::show(std::ostream& stream) {
  stream << "====================\nLoop Function params\n===================="
         << std::endl;
  stream << "arena_x= " << m_params->arena_x << std::endl;
  stream << "arena_y= " << m_params->arena_y << std::endl;
  stream << "nest_x= " << m_params->nest_x << std::endl;
  stream << "nest_y= " << m_params->nest_y << std::endl;
} /* show() */

NS_END(params, fordyca);
