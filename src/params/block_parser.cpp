/**
 * @file block_parser.cpp
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
#include <argos3/core/utility/configuration/argos_configuration.h>

#include "fordyca/params/block_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char block_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_parser::parse(const ticpp::Element& node) {
  ticpp::Element bnode =
      argos::GetNode(const_cast<ticpp::Element&>(node), kXMLRoot);
  XML_PARSE_PARAM(bnode, m_params, n_blocks);
  XML_PARSE_PARAM(bnode, m_params, dimension);
  XML_PARSE_PARAM(bnode, m_params, dist_model);
} /* parse() */

void block_parser::show(std::ostream& stream) const {
  stream << build_header() << XML_PARAM_STR(m_params, n_blocks) << std::endl
         << XML_PARAM_STR(m_params, dimension) << std::endl
         << XML_PARAM_STR(m_params, dist_model) << std::endl
         << build_footer();
} /* show() */

bool block_parser::validate(void) const {
  return !(0 == m_params.n_blocks || m_params.dimension <= 0.0 ||
           "" == m_params.dist_model);
} /* validate() */

NS_END(params, fordyca);
