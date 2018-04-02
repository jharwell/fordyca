/**
 * @file explore_for_cache_fsm.cpp
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
#include "fordyca/fsm/depth1/explore_for_cache_fsm.hpp"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/datatypes/color.h>
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/depth1/sensing_subsystem.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/params/fsm_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth1);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
explore_for_cache_fsm::explore_for_cache_fsm(
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
  insmod("explore_for_cache_fsm",
         rcppsw::er::er_lvl::DIAG,
         rcppsw::er::er_lvl::NOM);
}

HFSM_STATE_DEFINE_ND(explore_for_cache_fsm, start) {
  internal_event(ST_EXPLORE);
  return controller::foraging_signal::HANDLED;
}

__const HFSM_STATE_DEFINE_ND(explore_for_cache_fsm, finished) {
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE_ND(explore_for_cache_fsm, explore) {
  if (ST_EXPLORE != last_state()) {
    ER_DIAG("Executing ST_EXPLORE");
  }
  if (ST_NEW_DIRECTION == last_state()) {
    explore_time_reset();
  }
  if (m_sensors->cache_detected()) {
    ER_DIAG("Cache detected");
    internal_event(ST_FINISHED);
  }

  base_explore_fsm::explore_time_inc();

  /*
   * Check for nearby obstacles, and if so go into obstacle avoidance. Time
   * spent in collision avoidance still counts towards the direction change
   * threshold.
   */
  if (base_sensors()->threatening_obstacle_exists()) {
    argos::CVector2 force = kinematics().calc_avoidance_force();
    ER_DIAG("Found threatening obstacle: avoidance force=(%f, %f)@%f [%f]",
            force.GetX(),
            force.GetY(),
            force.Angle().GetValue(),
            force.Length());

    internal_event(ST_COLLISION_AVOIDANCE);
  } else if (explore_time() > base_explore_fsm::dir_change_thresh()) {
    argos::CRange<argos::CRadians> range(argos::CRadians(0.50),
                                         argos::CRadians(1.0));
    argos::CVector2 new_dir = randomize_vector_angle(argos::CVector2::X);
    internal_event(ST_NEW_DIRECTION,
                   rcppsw::make_unique<new_direction_data>(new_dir.Angle()));
  }

  /*
   * No obstacles nearby--all ahead full!
   */
  actuators()->set_rel_heading(argos::CVector2::X *
                               actuators()->differential_drive()->max_speed());
  return controller::foraging_signal::HANDLED;
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void explore_for_cache_fsm::init(void) {
  base_explore_fsm::init();
} /* init() */

void explore_for_cache_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* task_execute() */

bool explore_for_cache_fsm::task_running(void) const {
  return ST_START != current_state() && ST_FINISHED != current_state();
}

NS_END(depth1, fsm, fordyca);
