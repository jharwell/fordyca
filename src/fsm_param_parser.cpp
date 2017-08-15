/**
 * @file fsm_param_parser.cpp
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
#include "fordyca/params/fsm_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void fsm_param_parser::parse(argos::TConfigurationNode& node) {
  argos::TConfigurationNode fsm_node = argos::GetNode(node, "fsm");

  m_params.reset(new foraging_fsm_params);
  try {
      argos::GetNodeAttribute(fsm_node,
                              "unsuccessful_explore_dir_change",
                              m_params->times.unsuccessful_explore_dir_change);
  }
  catch (argos::CARGoSException& ex) {
    using namespace argos;
    THROW_ARGOSEXCEPTION_NESTED("Error initializing FSM parameters.", ex);
  }
} /* parse() */

void fsm_param_parser::show(std::ostream& stream) {
  stream << "FSM params\n====================" << std::endl;
  stream << "times.unsuccessful_explore_dir_change="
         << m_params->times.unsuccessful_explore_dir_change << std::endl;
} /* show() */

NS_END(params, fordyca);
