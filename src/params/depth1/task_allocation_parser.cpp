/**
 * @file task_allocation_parser.cpp
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
#include "fordyca/params/depth1/task_allocation_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth1);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char task_allocation_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void task_allocation_parser::parse(const ticpp::Element& node) {
  ticpp::Element tnode =
      argos::GetNode(const_cast<ticpp::Element&>(node), kXMLRoot);

  m_exec_parser.parse(tnode);
  m_estimate_parser.parse(tnode);
  m_params.executive = *m_exec_parser.parse_results();
  m_params.exec_estimates = *m_estimate_parser.parse_results();
} /* parse() */

void task_allocation_parser::show(std::ostream& stream) const {
  stream << build_header() << m_exec_parser << m_estimate_parser << std::endl;
} /* show() */

__pure bool task_allocation_parser::validate(void) const {
  return m_exec_parser.validate() && m_estimate_parser.validate();
} /* validate() */

NS_END(depth1, params, fordyca);
