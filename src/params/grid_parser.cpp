/**
 * @file grid_parser.cpp
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
#include "fordyca/params/grid_parser.hpp"
#include <argos3/core/utility/configuration/argos_configuration.h>

#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char grid_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void grid_parser::parse(const ticpp::Element& node) {
  ticpp::Element gnode = argos::GetNode(const_cast<ticpp::Element&>(node),
                                        kXMLRoot);
  std::vector<std::string> res;

  rcppsw::utils::line_parser parser(' ');
  res = parser.parse(node.GetAttribute("size"));

  m_params.resolution = std::atof(node.GetAttribute("resolution").c_str());
  m_params.lower.Set(0, 0);
  m_params.upper.Set(std::atoi(res[0].c_str()), std::atoi(res[1].c_str()));
} /* parse() */

void grid_parser::show(std::ostream& stream) const {
  stream << emit_header()
         << XML_PARAM_STR(m_params, resolution) << std::endl
         << m_params.lower << std::endl
         << m_params.upper << std::endl;
} /* show() */

__pure bool grid_parser::validate(void) const {
  if (!(m_params.resolution > 0.0)) {
    return false;
  }
  if (!(m_params.upper.GetX() > 0) || !(m_params.upper.GetY() > 0)) {
    return false;
  }
  return true;
} /* validate() */

NS_END(params, fordyca);
