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

  XML_PARSE_PARAM(enode, m_params, elow);
  XML_PARSE_PARAM(enode, m_params, ehigh);
  XML_PARSE_PARAM(enode, m_params, capacity);
  XML_PARSE_PARAM(enode, m_params, weight1);
  XML_PARSE_PARAM(enode, m_params, weight2);
  XML_PARSE_PARAM(enode, m_params, weight3);
  XML_PARSE_PARAM(enode, m_params, weight1C);
  XML_PARSE_PARAM(enode, m_params, weight2C);
  XML_PARSE_PARAM(enode, m_params, weight3C);
  XML_PARSE_PARAM(enode, m_params, EEE);
} /* parse() */

void grid_parser::show(std::ostream& stream) const {
  stream << build_header() << XML_PARAM_STR(m_params, elow) << std::endl
         << XML_PARAM_STR(m_params, ehigh) << std::endl
         << XML_PARAM_STR(m_params, capacity) << std::endl
         << XML_PARAM_STR(m_params, weight1) << std::endl
         << XML_PARAM_STR(m_params, weight2) << std::endl
         << XML_PARAM_STR(m_params, weight3) << std::endl
         << XML_PARAM_STR(m_params, weight1C) << std::endl
         << XML_PARAM_STR(m_params, weight2C) << std::endl
         << XML_PARAM_STR(m_params, weight3C) << std::endl
         << XML_PARAM_STR(m_params, EEE) << std::endl
         << build_footer();
} /* show() */

__rcsw_pure bool grid_parser::validate(void) const {
  CHECK(m_params->elow > 0.0);
  CHECK(m_params->ehigh > 0.0);
  CHECK(m_params->capacity > 0.0);
  return true;

error:
  return false;
} /* validate() */

NS_END(params, fordyca);
