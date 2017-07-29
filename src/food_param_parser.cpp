/**
 * @file food_param_parser.cpp
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
#include "fordyca/food_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
const struct food_params& food_param_parser::parse(argos::TConfigurationNode& node) {
  argos::TConfigurationNode food_node = argos::GetNode(node, "food");

  argos::GetNodeAttribute(food_node, "n_items", m_params.n_items);
  argos::GetNodeAttribute(food_node, "radius", m_params.square_radius);
  m_params.square_radius *= m_params.square_radius;
  argos::GetNodeAttribute(food_node, "energy_per_item", m_params.energy_per_item);
  return m_params;
} /* food_param_parser:parse() */

NS_END(fordyca);
