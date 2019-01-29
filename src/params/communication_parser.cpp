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
#include "fordyca/params/communication_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char communication_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void communication_parser::parse(const ticpp::Element& node) {
  ticpp::Element anode = get_node(const_cast<ticpp::Element&>(node), kXMLRoot);

  m_params =
      std::make_shared<std::remove_reference<decltype(*m_params)>::type>();
  XML_PARSE_ATTR(anode, m_params, on);
  XML_PARSE_ATTR(anode, m_params, mode);
  XML_PARSE_ATTR(anode, m_params, max_message_length);
  XML_PARSE_ATTR(anode, m_params, chance_to_send_communication);
  XML_PARSE_ATTR(anode, m_params, chance_to_recieve_communication);
} /* parse() */

void communication_parser::show(std::ostream& stream) const {
  stream << build_header();

  stream << XML_ATTR_STR(m_params, on)
         << XML_ATTR_STR(m_params, mode)
         << XML_ATTR_STR(m_params, max_message_length)
         << XML_ATTR_STR(m_params, chance_to_send_communication)
         << XML_ATTR_STR(m_params, chance_to_recieve_communication)
         << std::endl
         << build_footer();
} /* show() */

__rcsw_pure bool communication_parser::validate(void) const {
    return m_params->max_message_length >= 0 &&
           m_params->max_message_length <= 20 &&
           m_params->chance_to_send_communication >= 0.0 &&
           m_params->chance_to_send_communication <= 1.0 &&
           m_params->chance_to_recieve_communication >= 0.0 &&
           m_params->chance_to_recieve_communication <= 1.0;
} /* validate() */


NS_END(params, fordyca);
