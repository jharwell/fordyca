/**
 * @file energy_parser.cpp
 *
 * @copyright 2018 Anthony Chen/John Harwell, All rights reserved.
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
#include "fordyca/params/energy_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char energys_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void energy_parser::parse(const ticpp::Element& node) {
  ticpp::Element enode = get_node(const_cast<ticpp::Element&>(node), kXMLRoot);
  m_params =
      std::make_shared<std::remove_reference<decltype(*m_params)>::type>();

  XML_PARSE_PARAM(enode, m_params, max_energy);
  XML_PARSE_PARAM(enode, m_params, threshold);
} /* parse() */

void grid_parser::show(std::ostream& stream) const {
  stream << build_header() << XML_PARAM_STR(m_params, max_energy) << std::endl
         << XML_PARAM_STR(m_params, threshold)
         << build_footer();
} /* show() */

__rcsw_pure bool grid_parser::validate(void) const {
  CHECK(m_params->max_energy > 0.0);
  CHECK(m_params->threshold > 0.0);
  return true;

error:
  return false;
} /* validate() */

NS_END(params, fordyca);
