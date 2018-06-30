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
#include "fordyca/params/nest_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char nest_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nest_parser::parse(const ticpp::Element& node) {
  ticpp::Element nnode =
      get_node(const_cast<ticpp::Element&>(node), kXMLRoot);

  std::vector<std::string> res;
  rcppsw::utils::line_parser parser(' ');

  res = parser.parse(nnode.GetAttribute("center"));

  m_params = std::make_shared<std::remove_reference<decltype(*m_params)>::type>();
  m_params->center = argos::CVector2(std::atof(res[0].c_str()),
                                         std::atof(res[1].c_str()));
  res = parser.parse(nnode.GetAttribute("size"));
  m_params->xdim = std::atof(res[0].c_str());
  m_params->ydim = std::atof(res[1].c_str());
} /* parse() */

void nest_parser::show(std::ostream& stream) const {
  stream << build_header()
         << "xdim=" << m_params->xdim << std::endl
         << "ydim=" << m_params->ydim << std::endl
         << "center=" << m_params->center << std::endl
         << build_footer();
} /* show() */

bool nest_parser::validate(void) const {
  CHECK(m_params->center.GetX() > 0);
  CHECK(m_params->center.GetY() > 0);
  CHECK(m_params->xdim > 0);
  CHECK(m_params->ydim > 0);
  return true;

 error:
  return false;
} /* validate() */

NS_END(params, fordyca);
