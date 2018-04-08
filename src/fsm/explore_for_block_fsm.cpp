/**
 * @file explore_for_block_fsm.cpp
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
#include "fordyca/fsm/explore_for_block_fsm.hpp"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/datatypes/color.h>
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/params/fsm_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
namespace kinematics = rcppsw::robotics::kinematics;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
explore_for_block_fsm::explore_for_block_fsm(
    uint unsuccessful_dir_change_thresh,
    const std::shared_ptr<rcppsw::er::server>& server,
    const std::shared_ptr<controller::saa_subsystem>& saa)
    : base_explore_fsm(unsuccessful_dir_change_thresh,
                       server,
                       saa,
                       ST_MAX_STATES),
      HFSM_CONSTRUCT_STATE(collision_avoidance, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(new_direction, hfsm::top_state()),
      entry_collision_avoidance(),
      entry_new_direction(),
      entry_explore(),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(explore, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      mc_state_map{
          HFSM_STATE_MAP_ENTRY_EX(&start),
          HFSM_STATE_MAP_ENTRY_EX_ALL(&explore, nullptr, &entry_explore, nullptr),
          HFSM_STATE_MAP_ENTRY_EX_ALL(&collision_avoidance,
                                      nullptr,
                                      &entry_collision_avoidance,
                                      nullptr),
          HFSM_STATE_MAP_ENTRY_EX_ALL(&new_direction,
                                      nullptr,
                                      &entry_new_direction,
                                      nullptr),
          HFSM_STATE_MAP_ENTRY_EX(&finished)} {
  insmod("explore_for_block_fsm",
         rcppsw::er::er_lvl::DIAG,
         rcppsw::er::er_lvl::NOM);
}

HFSM_STATE_DEFINE_ND(explore_for_block_fsm, start) {
  internal_event(ST_EXPLORE);
  return controller::foraging_signal::HANDLED;
}

__const HFSM_STATE_DEFINE_ND(explore_for_block_fsm, finished) {
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE_ND(explore_for_block_fsm, explore) {
  if (ST_EXPLORE != last_state()) {
    ER_DIAG("Executing ST_EXPLORE");
  }
  if (ST_NEW_DIRECTION == last_state()) {
    explore_time_reset();
  }
  if (base_foraging_fsm::base_sensors()->block_detected()) {
    internal_event(ST_FINISHED);
  }

  base_explore_fsm::explore_time_inc();
  saa_subsystem()->steering_force().wander();
  argos::CVector2 obs = base_sensors()->find_closest_obstacle();
  saa_subsystem()->steering_force().avoidance(obs);

  if (base_sensors()->threatening_obstacle_exists()) {
    ER_DIAG("Found threatening obstacle: (%f, %f)@%f [%f]",
           obs.GetX(),
           obs.GetY(),
           obs.Angle().GetValue(),
           obs.Length());
    saa_subsystem()->apply_steering_force();
  } else {
    ER_DIAG("No threatening obstacle found");
    argos::CVector2 force = saa_subsystem()->steering_force().value();
      /*
       * This can be 0 if the wander force is not active this timestep.
       */
      if (force.Length() >= std::numeric_limits<double>::epsilon()) {
        saa_subsystem()->steering_force().value() *= 0.7;
        saa_subsystem()->apply_steering_force();
      }
  }
  return controller::foraging_signal::HANDLED;
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void explore_for_block_fsm::init(void) {
  base_explore_fsm::init();
} /* init() */

void explore_for_block_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* task_execute() */

bool explore_for_block_fsm::task_running(void) const {
  return ST_START != current_state() && ST_FINISHED != current_state();
}

NS_END(fsm, fordyca);
