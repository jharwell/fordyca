/**
 * @file phototaxis_force_parser.cpp
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
#include <argos3/core/utility/configuration/argos_configuration.h>

#include "fordyca/params/phototaxis_force_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char phototaxis_force_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void phototaxis_force_parser::parse(const ticpp::Element& node) {
  ticpp::Element pnode =
      argos::GetNode(const_cast<ticpp::Element&>(node), kXMLRoot);
  XML_PARSE_PARAM(pnode, m_params, max);
} /* parse() */

void phototaxis_force_parser::show(std::ostream& stream) const {
  stream << build_header()
         << XML_PARAM_STR(m_params, max) << std::endl
         << build_footer();
} /* show() */

__pure bool phototaxis_force_parser::validate(void) const {
  return m_params.max >= 0;
} /* validate() */

NS_END(params, fordyca);