/**
 * @file random_explore_behavior.cpp
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
#include "fordyca/controller/random_explore_behavior.hpp"

/*******************************************************************************
 * Namespaces
******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
random_explore_behavior::random_explore_behavior(
    const std::shared_ptr<rcppsw::er::server>& server,
    controller::saa_subsystem* const saa)
    : explore_behavior(server, saa) {
  insmod("random_explore_behavior",
         rcppsw::er::er_lvl::DIAG,
         rcppsw::er::er_lvl::NOM);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void random_explore_behavior::execute(void) {
  argos::CVector2 obs = saa_subsystem()->sensing()->find_closest_obstacle();
  saa_subsystem()->steering_force().avoidance(obs);
  saa_subsystem()->steering_force().wander();

  if (saa_subsystem()->sensing()->threatening_obstacle_exists()) {
    ER_DIAG("Found threatening obstacle: (%f, %f)@%f [%f]",
            obs.GetX(),
            obs.GetY(),
            obs.Angle().GetValue(),
            obs.Length());
    saa_subsystem()->apply_steering_force(std::make_pair(false, false));
    saa_subsystem()->actuation()->leds_set_color(utils::color::kRED);
  } else {
    ER_DIAG("No threatening obstacle found");
    saa_subsystem()->actuation()->leds_set_color(utils::color::kMAGENTA);
    argos::CVector2 force = saa_subsystem()->steering_force().value();
    /*
     * This can be 0 if the wander force is not active this timestep.
     */
    if (force.Length() >= std::numeric_limits<double>::epsilon()) {
      saa_subsystem()->steering_force().value(
          saa_subsystem()->steering_force().value() * 0.7);
      saa_subsystem()->apply_steering_force(std::make_pair(false, false));
    }
  }
} /* execute() */

NS_END(controller, fordyca);
