/**
 * @file steering_force2D_parser.cpp
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
#include "fordyca/params/steering_force2D_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void steering_force2D_parser::parse(const ticpp::Element& node) {
  force_calculator_xml_parser::parse(node);
  m_params =
      std::make_shared<std::remove_reference<decltype(*m_params)>::type>();
  *std::static_pointer_cast<steering::force_calculator_params>(m_params) =
      *force_calculator_xml_parser::parse_results();

  ticpp::Element snode = node_get(const_cast<ticpp::Element&>(node), kXMLRoot);
  m_phototaxis.parse(snode);
  m_params->phototaxis = *m_phototaxis.parse_results();
} /* parse() */

void steering_force2D_parser::show(std::ostream& stream) const {
  force_calculator_xml_parser::show(stream);
  stream << m_phototaxis << build_footer();
} /* show() */

__rcsw_pure bool steering_force2D_parser::validate(void) const {
  return force_calculator_xml_parser::validate() && m_phototaxis.validate();
} /* validate() */

NS_END(params, fordyca);
