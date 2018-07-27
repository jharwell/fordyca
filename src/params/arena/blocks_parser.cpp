/**
 * @file block_parser.cpp
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
#include "fordyca/params/arena/blocks_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, arena);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char blocks_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void blocks_parser::parse(const ticpp::Element& node) {
  ticpp::Element bnode =
      get_node(const_cast<ticpp::Element&>(node), kXMLRoot);
  m_params =
      std::make_shared<std::remove_reference<decltype(*m_params)>::type>();

  m_manipulation_penalty.parse(get_node(const_cast<ticpp::Element&>(bnode),
                                        "manipulation_penalty"));
  m_dist.parse(bnode);
  m_params->manipulation_penalty = *m_manipulation_penalty.parse_results();
  m_params->dist = *m_dist.parse_results();
} /* parse() */

void blocks_parser::show(std::ostream& stream) const {
  stream << build_header()
         << m_manipulation_penalty
         << m_dist
         << build_footer();
} /* show() */

__rcsw_pure bool blocks_parser::validate(void) const {
  CHECK(true == m_manipulation_penalty.validate());
  CHECK(true == m_dist.validate());
  return true;

error:
  return false;
} /* validate() */

NS_END(arena, params, fordyca);
