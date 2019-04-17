/**
 * @file block_sel_matrix_parser.cpp
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
#include "fordyca/params/block_sel/block_sel_matrix_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, block_sel);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char block_sel_matrix_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_sel_matrix_parser::parse(const ticpp::Element& node) {
  ticpp::Element cnode = node_get(node, kXMLRoot);
  m_params =
      std::make_shared<std::remove_reference<decltype(*m_params)>::type>();

  rcppsw::utils::line_parser parser(' ');
  std::string val;
  std::vector<std::string> res;
  res = parser.parse(cnode.GetAttribute("nest"));
  m_params->nest.set(std::atof(res[0].c_str()), std::atof(res[1].c_str()));

  m_priorities.parse(cnode);
  m_params->priorities = *m_priorities.parse_results();
} /* parse() */

NS_END(block_sel, params, fordyca);
