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
      get_node(const_cast<ticpp::Element&>(node), kXMLRoot);
  m_params =
      std::make_shared<std::remove_reference<decltype(*m_params)>::type>();

  XML_PARSE_PARAM(bnode, m_params, dimension);
  m_manipulation_penalty.parse(get_node(const_cast<ticpp::Element&>(bnode),
                                        "manipulation_penalty"));
  m_params->manipulation_penalty = *m_manipulation_penalty.parse_results();
} /* parse() */

void block_parser::show(std::ostream& stream) const {
  stream << build_header() << XML_PARAM_STR(m_params, dimension) << std::endl
         << m_manipulation_penalty
         << build_footer();
} /* show() */

__rcsw_pure bool block_parser::validate(void) const {
  CHECK(m_params->dimension > 0.0);
  CHECK(true == m_manipulation_penalty.validate());
  return true;

error:
  return false;
} /* validate() */

NS_END(params, fordyca);
