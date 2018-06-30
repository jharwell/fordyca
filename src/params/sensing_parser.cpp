/**
 * @file sensing_parser.cpp
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
#include "fordyca/params/sensing_parser.hpp"
#include <argos3/core/utility/configuration/argos_configuration.h>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char sensing_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void sensing_parser::parse(const ticpp::Element& node) {
  ticpp::Element snode =
      argos::GetNode(const_cast<ticpp::Element&>(node), kXMLRoot);

  m_params =
  std::make_shared<std::remove_reference<decltype(*m_params)>::type>();
  m_proximity_parser.parse(snode);
  m_params->proximity = *m_proximity_parser.parse_results();
} /* parse() */

void sensing_parser::show(std::ostream& stream) const {
  stream << build_header()
         << m_proximity_parser
         << build_footer();
} /* show() */

__rcsw_pure bool sensing_parser::validate(void) const {
  return m_proximity_parser.validate();
} /* validate() */

NS_END(params, fordyca);
