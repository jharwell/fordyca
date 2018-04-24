/**
 * @file base_explore_fsm.cpp
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
#include "fordyca/fsm/base_explore_fsm.hpp"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/datatypes/color.h>
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/base_sensing_subsystem.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/params/fsm_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_explore_fsm::base_explore_fsm(
    const std::shared_ptr<rcppsw::er::server>& server,
    const std::shared_ptr<controller::saa_subsystem>& saa,
    uint8_t max_states)
    : base_foraging_fsm(server, saa, max_states), entry_explore() {
  insmod("base_explore_fsm", rcppsw::er::er_lvl::DIAG, rcppsw::er::er_lvl::NOM);
}

/*******************************************************************************
 * States
 ******************************************************************************/
HFSM_ENTRY_DEFINE_ND(base_explore_fsm, entry_explore) {
  base_foraging_fsm::actuators()->leds_set_color(argos::CColor::MAGENTA);
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void base_explore_fsm::run(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* run() */

void base_explore_fsm::random_explore(void) {
  argos::CVector2 obs = base_sensors()->find_closest_obstacle();
  saa_subsystem()->steering_force().avoidance(obs);
  saa_subsystem()->steering_force().wander();

  if (base_sensors()->threatening_obstacle_exists()) {
    ER_DIAG("Found threatening obstacle: (%f, %f)@%f [%f]",
            obs.GetX(),
            obs.GetY(),
            obs.Angle().GetValue(),
            obs.Length());
    saa_subsystem()->apply_steering_force(std::make_pair(false, false));
    saa_subsystem()->actuation()->leds_set_color(argos::CColor::RED);
  } else {
    ER_DIAG("No threatening obstacle found");
    saa_subsystem()->actuation()->leds_set_color(argos::CColor::MAGENTA);
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
} /* random_explore() */

void base_explore_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* task_execute() */

NS_END(fsm, fordyca);
