/**
 * @file unpartitioned_task_loop_functions.cpp
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
#include <limits>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include "fordyca/support/unpartitioned_task_loop_functions.hpp"
#include "fordyca/controller/unpartitioned_task_controller.hpp"
#include "fordyca/events/block_drop.hpp"
#include "fordyca/events/block_pickup.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void unpartitioned_task_loop_functions::Init(argos::TConfigurationNode& node) {
  random_foraging_loop_functions::Init(node);

  ER_NOM("Initializing unpartitioned_task loop functions");

  /* configure robots */
  argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
  for (argos::CSpace::TMapPerType::iterator it = footbots.begin();
       it != footbots.end();
       ++it) {
    argos::CFootBotEntity& robot = *argos::any_cast<argos::CFootBotEntity*>(
        it->second);
    controller::unpartitioned_task_controller& controller =
        dynamic_cast<controller::unpartitioned_task_controller&>(
            robot.GetControllableEntity().GetController());
    const struct loop_functions_params * l_params =
        static_cast<const struct loop_functions_params*>(
            repo()->get_params("loop_functions"));

    controller.display_los(l_params->display_robot_los);
    set_robot_los(robot);
  } /* for(it..) */
  ER_NOM("unpartitioned_task loop functions initialization finished");
}

void unpartitioned_task_loop_functions::pre_step_iter(argos::CFootBotEntity& robot) {
    controller::unpartitioned_task_controller& controller =
        dynamic_cast<controller::unpartitioned_task_controller&>(
        robot.GetControllableEntity().GetController());

    /* Send the robot its new line of sight */
    set_robot_los(robot);
    set_robot_tick(robot);

    /* get stats from this robot before its state changes */
    collector()->collect_from_robot(controller);

    if (controller.is_carrying_block()) {
      if (controller.in_nest()) {
        /*
         * Get stats from carried block before it's dropped and its state
         * changes.
         */
        collector()->collect_from_block(*controller.block());

        /* Update arena map state due to a block nest drop */
        events::block_drop drop_op(rcppsw::common::g_server,
                                       controller.block());
        map()->accept(drop_op);

        /* Actually drop the block */
        controller.visitor::visitable<controller::unpartitioned_task_controller>::accept(drop_op);

        /* The floor texture must be updated */
        floor()->SetChanged();
      }
    } else { /* The foot-bot has no block item */
      if (!controller.in_nest() && controller.is_searching_for_block() &&
          controller.block_detected()) {
        /* Check whether the foot-bot is actually on a block */
        int block = robot_on_block(robot);
        if (-1 != block) {
          events::block_pickup pickup_op(rcppsw::common::g_server,
                                             &map()->blocks()[block],
                                             robot_id(robot));
          controller.visitor::visitable<controller::unpartitioned_task_controller>::accept(pickup_op);
          map()->accept(pickup_op);
          controller.publish_fsm_event(controller::foraging_signal::BLOCK_ACQUIRED);

          /* The floor texture must be updated */
          floor()->SetChanged();
        }
      }
    }
} /* pre_step_iter() */

void unpartitioned_task_loop_functions::PreStep() {
  argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");

  for (argos::CSpace::TMapPerType::iterator it = footbots.begin();
       it != footbots.end();
       ++it) {
    argos::CFootBotEntity& robot = *argos::any_cast<argos::CFootBotEntity*>(
        it->second);
    pre_step_iter(robot);
  } /* for(it..) */
  random_foraging_loop_functions::pre_step_final();
} /* PreStep() */

void unpartitioned_task_loop_functions::set_robot_los(argos::CFootBotEntity& robot) {
  argos::CVector2 pos;
  pos.Set(const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
          const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

  representation::discrete_coord robot_loc =
      representation::real_to_discrete_coord(
          std::pair<double, double>(pos.GetX(), pos.GetY()),
          map()->grid_resolution());
  controller::unpartitioned_task_controller& controller =
      dynamic_cast<controller::unpartitioned_task_controller&>(
          robot.GetControllableEntity().GetController());
  std::unique_ptr<representation::line_of_sight> new_los =
      rcppsw::make_unique<representation::line_of_sight>(
          map()->subgrid(robot_loc.first, robot_loc.second, 1),
          robot_loc);
  controller.los(new_los);
  controller.robot_loc(pos);
} /* set_robot_los() */

void unpartitioned_task_loop_functions::set_robot_tick(argos::CFootBotEntity& robot) {
  controller::unpartitioned_task_controller& controller =
      dynamic_cast<controller::unpartitioned_task_controller&>(
          robot.GetControllableEntity().GetController());
  controller.tick(GetSpace().GetSimulationClock() + 1); /* for next timestep */
} /* set_robot_tic() */

using namespace argos;
REGISTER_LOOP_FUNCTIONS(unpartitioned_task_loop_functions, "unpartitioned_task_loop_functions");

NS_END(support, fordyca);
