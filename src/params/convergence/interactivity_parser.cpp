/**
 * @file interactivity_parser.cpp
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
#include "fordyca/params/convergence/interactivity_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, convergence);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char interactivity_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void interactivity_parser::parse(const ticpp::Element& node) {
    ticpp::Element mnode = get_node(const_cast<ticpp::Element&>(node), kXMLRoot);
    XML_PARSE_ATTR(mnode, m_params, enable);
} /* parse() */

NS_END(convergence, params, fordyca);
