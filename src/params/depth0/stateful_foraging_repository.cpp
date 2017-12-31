/**
 * @file depth0_foraging_repository.cpp
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
#include "fordyca/params/depth0/stateful_foraging_repository.hpp"
#include "fordyca/params/actuator_parser.hpp"
#include "fordyca/params/depth0/perceived_arena_map_parser.hpp"
#include "fordyca/params/fsm_parser.hpp"
#include "fordyca/params/sensor_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateful_foraging_repository::stateful_foraging_repository(void) {
  register_parser<actuator_parser>("actuators");
  register_parser<sensor_parser>("sensors");
  register_parser<fsm_parser>("fsm");
  register_parser<perceived_arena_map_parser>("perceived_arena_map");
}

NS_END(depth0, params, fordyca);
