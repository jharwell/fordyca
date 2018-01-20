/**
 * @file fsm_parser.cpp
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
#include "fordyca/params/fsm_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void fsm_parser::parse(argos::TConfigurationNode& node) {
  argos::TConfigurationNode fsm_node = argos::GetNode(node, "fsm");

  m_params = rcppsw::make_unique<fsm_params>();

  argos::GetNodeAttribute(fsm_node,
                          "unsuccessful_explore_dir_change",
                          m_params->times.unsuccessful_explore_dir_change);
  argos::GetNodeAttribute(fsm_node,
                          "frequent_collision_thresh",
                          m_params->times.frequent_collision_thresh);
  argos::GetNodeAttribute(fsm_node,
                          "speed_throttle_block_carry",
                          m_params->speed_throttling.block_carry);

  rcppsw::utils::line_parser parser(' ');
  std::vector<std::string> res;
  res = parser.parse(fsm_node.GetAttribute("nest"));
  m_params->nest_center.Set(std::atof(res[0].c_str()),
                            std::atof(res[1].c_str()));
} /* parse() */

void fsm_parser::show(std::ostream& stream) {
  stream << "====================\nFSM params\n====================\n";
  stream << "times.unsuccessful_explore_dir_change="
         << m_params->times.unsuccessful_explore_dir_change << std::endl;
  stream << "times.frequent_collision_thresh="
         << m_params->times.frequent_collision_thresh << std::endl;
  stream << "speed_throttling.block_carry="
         << m_params->speed_throttling.block_carry << std::endl;
  stream << "nest_center=" << m_params->nest_center << std::endl;
} /* show() */

__pure bool fsm_parser::validate(void) {
  return (m_params->nest_center.GetX() > 0) &&
         (m_params->nest_center.GetY() > 0);
} /* validate() */

NS_END(params, fordyca);
