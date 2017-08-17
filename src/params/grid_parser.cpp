/**
 * @file grid_parser.cpp
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
#include "fordyca/params/grid_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void grid_parser::parse(argos::TConfigurationNode& node) {
  m_params.reset(new struct grid_params);
  ticpp::Node *arena = node.NextSibling("arena");
  std::vector<std::string> res;

  rcppsw::utils::line_parser parser(' ');
  res = parser.parse(arena->ToElement()->GetAttribute("size"));

  m_params->resolution = std::atof(
      argos::GetNode(node, "grid").GetAttribute("resolution").c_str());
  m_params->lower.Set(-std::atoi(res[0].c_str())/2.0 + 0.3,
                      -std::atoi(res[1].c_str())/2.0 + 0.3);
  m_params->upper.Set(std::atoi(res[0].c_str())/2.0 - 0.3,
                      std::atoi(res[1].c_str())/2.0 - 0.3);

  argos::TConfigurationNode bnode = argos::GetNode(node, "blocks");
  argos::GetNodeAttribute(bnode, "n_blocks", m_params->block.n_blocks);
  argos::GetNodeAttribute(bnode, "dimension", m_params->block.dimension);
  argos::GetNodeAttribute(bnode, "dist_model", m_params->block.dist_model);
  argos::GetNodeAttribute(bnode, "respawn", m_params->block.respawn);
} /* parse() */

void grid_parser::show(std::ostream& stream) {
  stream << "====================\nGrid params\n====================\n";
  stream << "resolution=" << m_params->resolution << std::endl;
  stream << "lower=" << m_params->lower << std::endl;
  stream << "upper=" << m_params->upper << std::endl;
  stream << "block.n_items=" << m_params->block.n_blocks << std::endl;
  stream << "block.dimension=" << m_params->block.dimension << std::endl;
  stream << "block.dist_model=" << m_params->block.dist_model << std::endl;
  stream << "block.respawn=" << m_params->block.respawn << std::endl;
} /* show() */

NS_END(params, fordyca);