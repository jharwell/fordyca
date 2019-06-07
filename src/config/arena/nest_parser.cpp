/**
 * @file nest_parser.cpp
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
#include "fordyca/config/arena/nest_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, arena);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char nest_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nest_parser::parse(const ticpp::Element& node) {
  ticpp::Element nnode = node_get(node, kXMLRoot);

  std::vector<std::string> res;
  rcppsw::utils::line_parser parser(' ');

  res = parser.parse(nnode.GetAttribute("center"));

  m_config =
      std::make_shared<std::remove_reference<decltype(*m_config)>::type>();
  m_config->center =
      rmath::vector2d(std::atof(res[0].c_str()), std::atof(res[1].c_str()));
  res = parser.parse(nnode.GetAttribute("size"));
  m_config->dims.x(std::atof(res[0].c_str()));
  m_config->dims.y(std::atof(res[1].c_str()));
} /* parse() */

void nest_parser::show(std::ostream& stream) const {
  stream << build_header() << "dims=" << m_config->dims << std::endl
         << "center=" << m_config->center << std::endl
         << build_footer();
} /* show() */

__rcsw_pure bool nest_parser::validate(void) const {
  CHECK(m_config->center.x() > 0);
  CHECK(m_config->center.y() > 0);
  CHECK(m_config->dims.x() > 0);
  CHECK(m_config->dims.y() > 0);
  return true;

error:
  return false;
} /* validate() */

NS_END(arena, config, fordyca);
