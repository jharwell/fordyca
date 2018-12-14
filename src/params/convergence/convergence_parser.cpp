/**
 * @file convergence_parser.cpp
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
#include "fordyca/params/convergence/convergence_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, convergence);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char convergence_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void convergence_parser::parse(const ticpp::Element& node) {
  ticpp::Element cnode = get_node(const_cast<ticpp::Element&>(node), kXMLRoot);

  m_pos_entropy.parse(cnode);
  m_params->pos_entropy = *m_pos_entropy.parse_results();
  m_interactivity.parse(cnode);
  m_params->interactivity = *m_interactivity.parse_results();
  m_ang_order.parse(cnode);
  m_params->ang_order = *m_ang_order.parse_results();
} /* parse() */

__rcsw_const bool convergence_parser::validate(void) const {
  return m_pos_entropy.validate() && m_interactivity.validate();
} /* validate() */

NS_END(convergence, params, fordyca);
