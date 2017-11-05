/**
 * @file depth1_foraging_loop_functions.cpp
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
#include "fordyca/support/depth1_foraging_loop_functions.hpp"
#include <limits>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include "fordyca/controller/depth1_foraging_controller.hpp"
#include "fordyca/events/nest_block_drop.hpp"
#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/params/loop_functions_params.hpp"
#include "fordyca/representation/line_of_sight.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void depth1_foraging_loop_functions::Init(argos::TConfigurationNode& node) {
  memory_foraging_loop_functions::Init(node);

  ER_NOM("Initializing depth1_foraging loop functions");

  ER_NOM("depth1_foraging loop functions initialization finished");
}

void depth1_foraging_loop_functions::pre_step_iter(argos::CFootBotEntity& robot) {
    controller::depth1_foraging_controller& controller =
        dynamic_cast<controller::depth1_foraging_controller&>(
        robot.GetControllableEntity().GetController());

    /* Send the robot its new line of sight */
    memory_foraging_loop_functions::set_robot_los(robot);
    memory_foraging_loop_functions::set_robot_tick(robot);

    if (controller.is_carrying_block()) {
      memory_foraging_loop_functions::handle_block_drop(controller);
    } else { /* The foot-bot has no block item */
      memory_foraging_loop_functions::handle_block_pickup(robot);
    }
} /* pre_step_iter() */

argos::CColor depth1_foraging_loop_functions::GetFloorColor(
    const argos::CVector2& plane_pos) {

  /* The nest is a light gray */
  if (nest_xrange().WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetX()) &&
      nest_yrange().WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetY())) {
    return argos::CColor::GRAY70;
  }

  /*
   * Blocks are inside caches, so display the cache the point is inside FIRST,
   * so that you don't have blocks renderin inside of caches.
   */
  for (size_t i = 0; i < map()->caches().size(); ++i) {
    if (map()->caches()[i].contains_point(plane_pos)) {
      return argos::CColor::GRAY40;
    }
  } /* for(i..) */

  for (size_t i = 0; i < map()->blocks().size(); ++i) {
    if (map()->blocks()[i].contains_point(plane_pos)) {
      return argos::CColor::BLACK;
    }
  } /* for(i..) */

  return argos::CColor::WHITE;
} /* GetFloorColor() */

void depth1_foraging_loop_functions::PreStep() {
  argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");

  for (argos::CSpace::TMapPerType::iterator it = footbots.begin();
       it != footbots.end();
       ++it) {
    argos::CFootBotEntity& robot = *argos::any_cast<argos::CFootBotEntity*>(
        it->second);
    pre_step_iter(robot);
  } /* for(it..) */
  memory_foraging_loop_functions::pre_step_final();
} /* PreStep() */

using namespace argos;
REGISTER_LOOP_FUNCTIONS(depth1_foraging_loop_functions, "depth1_foraging_loop_functions");

NS_END(support, fordyca);
