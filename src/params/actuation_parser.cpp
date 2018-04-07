/**
 * @file actuation_parser.cpp
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
#include "fordyca/params/actuation_parser.hpp"
#include <argos3/core/utility/configuration/argos_configuration.h>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char actuation_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void actuation_parser::parse(const ticpp::Element& node) {
  ticpp::Element anode =
      argos::GetNode(const_cast<ticpp::Element&>(node), kXMLRoot);
  m_wheels.parse(anode);
  m_steering.parse(anode);
  m_throttling.parse(anode);
  m_params.wheels = *m_wheels.parse_results();
  m_params.steering = *m_steering.parse_results();
  m_params.throttling = *m_throttling.parse_results();
} /* parse() */

void actuation_parser::show(std::ostream& stream) const {
  stream << build_header()
         << m_wheels << m_steering << m_throttling
         << build_footer();
} /* show() */

__pure bool actuation_parser::validate(void) const {
  return m_wheels.validate() && m_steering.validate() &&
      m_throttling.validate();
} /* validate() */

NS_END(params, fordyca);
