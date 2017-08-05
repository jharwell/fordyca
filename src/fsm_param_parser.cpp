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
#include "fordyca/fsm_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void fsm_param_parser::parse(argos::TConfigurationNode& node) {
  argos::TConfigurationNode fsm_node = argos::GetNode(node, "fsm");

  m_params.reset(new social_fsm_params);
  try {
       argos::GetNodeAttribute(fsm_node, "initial_rest_to_explore_prob", m_params->initial_rest_to_explore_prob);
      argos::GetNodeAttribute(fsm_node, "initial_explore_to_rest_prob", m_params->initial_explore_to_rest_prob);
      argos::GetNodeAttribute(fsm_node, "food_rule_explore_to_rest_delta_prob", m_params->deltas.food_rule_explore_to_rest);
      argos::GetNodeAttribute(fsm_node, "food_rule_rest_to_explore_delta_prob", m_params->deltas.food_rule_rest_to_explore);
      argos::GetNodeAttribute(fsm_node, "collision_rule_explore_to_rest_delta_prob", m_params->deltas.collision_rule_explore_to_rest);
      argos::GetNodeAttribute(fsm_node, "social_rule_rest_to_explore_delta_prob", m_params->deltas.social_rule_rest_to_explore);
      argos::GetNodeAttribute(fsm_node, "social_rule_explore_to_rest_delta_prob", m_params->deltas.social_rule_explore_to_rest);
      argos::GetNodeAttribute(fsm_node, "minimum_resting_time", m_params->times.min_rested);
      argos::GetNodeAttribute(fsm_node, "maximum_unsuccessful_explore_time", m_params->times.max_unsuccessful_explore);
      argos::GetNodeAttribute(fsm_node, "minimum_search_for_place_in_nest_time", m_params->times.min_search_for_place_in_nest);

   }
     catch (argos::CARGoSException& ex) {
       using namespace argos;
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller state parameters.", ex);
   }
} /* parse() */

void fsm_param_parser::print(std::ostream& stream) {
  stream << "FSM params\n====================" << std::endl;
  stream << "initial_rest_to_explore_prob=" << m_params->initial_rest_to_explore_prob << std::endl;
  stream << "initial_explore_to_rest_prob=" << m_params->initial_explore_to_rest_prob << std::endl;

  stream << "deltas.food_rule_explore_to_rest=" << m_params->deltas.food_rule_explore_to_rest << std::endl;
  stream << "deltas.food_rule_rest_to_explore=" << m_params->deltas.food_rule_rest_to_explore << std::endl;
  stream << "deltas.collision_rule_explore_to_rest=" << m_params->deltas.collision_rule_explore_to_rest << std::endl;
  stream << "deltas.social_rule_explore_to_rest=" << m_params->deltas.social_rule_explore_to_rest << std::endl;
  stream << "deltas.social_rule_rest_to_explore=" << m_params->deltas.social_rule_rest_to_explore << std::endl;
  stream << "times.min_rested=" << m_params->times.min_rested << std::endl;
  stream << "times.max_unsuccessful_explore=" << m_params->times.max_unsuccessful_explore << std::endl;
  stream << "times.min_search_for_place_in_nest=" << m_params->times.min_search_for_place_in_nest << std::endl;
} /* print() */

NS_END(fordyca);
