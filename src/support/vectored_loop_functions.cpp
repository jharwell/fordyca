/**
 * @file vectored_loop_functions.cpp
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
#include "fordyca/support/vectored_loop_functions.hpp"
#include "fordyca/controller/vectored_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void vectored_loop_functions::Init(argos::TConfigurationNode& node) {
  base_loop_functions::Init(node);

  /* configure robots */
  argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
  for (argos::CSpace::TMapPerType::iterator it = footbots.begin();
       it != footbots.end();
       ++it) {
    argos::CFootBotEntity& robot = *argos::any_cast<argos::CFootBotEntity*>(
        it->second);
    controller::vectored_controller& controller =
        dynamic_cast<controller::vectored_controller&>(
            robot.GetControllableEntity().GetController());
    const struct loop_functions_params * l_params =
        static_cast<const struct loop_functions_params*>(
            repo()->get_params("loop_functions"));

    controller.display_los(l_params->display_robot_los);
    set_robot_los(robot);
  } /* for(it..) */
}

void vectored_loop_functions::PreStep() {
  argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");

  for (argos::CSpace::TMapPerType::iterator it = footbots.begin();
       it != footbots.end();
       ++it) {
    argos::CFootBotEntity& robot = *argos::any_cast<argos::CFootBotEntity*>(
        it->second);

    /* Send the robot its new line of sight */
    set_robot_los(robot);
    set_robot_tick(robot);
    base_loop_functions::pre_step_iter(robot);
  } /* for(it..) */
  base_loop_functions::pre_step_final();
} /* PreStep() */

void vectored_loop_functions::set_robot_los(argos::CFootBotEntity& robot) {
  argos::CVector2 pos;
  pos.Set(const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
          const_cast<argos::CFootBotEntity&>(robot).GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
  representation::discrete_coord robot_loc =
      representation::real_to_discrete_coord(
          std::pair<double, double>(pos.GetX(), pos.GetY()),
          map()->grid_resolution());
  controller::vectored_controller& controller =
      dynamic_cast<controller::vectored_controller&>(
          robot.GetControllableEntity().GetController());
  std::unique_ptr<representation::line_of_sight> new_los =
      rcppsw::make_unique<representation::line_of_sight>(
          map()->subgrid(pos.GetX(), pos.GetY(),
                         map()->grid_resolution()),
          robot_loc);
  controller.los(new_los);
} /* set_robot_los() */

void vectored_loop_functions::set_robot_tick(argos::CFootBotEntity& robot) {
  controller::vectored_controller& controller =
      dynamic_cast<controller::vectored_controller&>(
          robot.GetControllableEntity().GetController());
  controller.tick(GetSpace().GetSimulationClock() + 1); /* for next timestep */
} /* set_robot_tic() */

using namespace argos;
REGISTER_LOOP_FUNCTIONS(vectored_loop_functions, "vectored_loop_functions");

NS_END(support, fordyca);
