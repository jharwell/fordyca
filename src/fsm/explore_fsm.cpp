/**
 * @file explore_fsm.cpp
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
#include "fordyca/fsm/explore_fsm.hpp"
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include "fordyca/params/fsm_params.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/controller/sensor_manager.hpp"
#include "fordyca/controller/actuator_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
explore_fsm::explore_fsm(
    double unsuccessful_dir_change_thresh,
    const std::shared_ptr<rcppsw::common::er_server>& server,
    const std::shared_ptr<controller::sensor_manager>& sensors,
    const std::shared_ptr<controller::actuator_manager>& actuators) :
    base_foraging_fsm(server, sensors, actuators, ST_MAX_STATES),
    HFSM_CONSTRUCT_STATE(collision_avoidance, hfsm::top_state()),
    entry_collision_avoidance(),
    HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(new_direction, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(explore, hfsm::top_state()),
    entry_new_direction(),
    entry_explore(),
    mc_unsuccessful_dir_change(unsuccessful_dir_change_thresh),
    m_rng(argos::CRandom::CreateRNG("argos")),
    m_state(),
    m_new_dir() {
  insmod("explore_fsm",
         rcppsw::common::er_lvl::DIAG,
         rcppsw::common::er_lvl::NOM);
}

HFSM_STATE_DEFINE(explore_fsm, start, state_machine::no_event_data) {
  internal_event(ST_EXPLORE);
  return controller::foraging_signal::HANDLED;
}
HFSM_STATE_DEFINE(explore_fsm, explore, state_machine::event_data) {
  if (ST_EXPLORE != last_state()) {
    ER_DIAG("Executing ST_EXPLORE");
  }
  if (base_foraging_fsm::sensors()->block_detected()) {
    ER_NOM("Block detected");
    return controller::foraging_signal::BLOCK_LOCATED;
  } else if (base_foraging_fsm::sensors()->cache_detected()) {
    ER_NOM("Cache detected");
    return controller::foraging_signal::CACHE_LOCATED;
  }

  explore_time_inc();

  /*
   * Check for nearby obstacles, and if so go into obstacle avoidance. Time spent
   * in collision avoidance still counts towards the direction change threshold.
   */
  if (base_foraging_fsm::sensors()->calc_diffusion_vector(NULL)) {
    internal_event(ST_COLLISION_AVOIDANCE);
  } else if (explore_time() > mc_unsuccessful_dir_change) {
    argos::CRange<argos::CRadians> range(argos::CRadians(0.50),
                                         argos::CRadians(1.0));
    argos::CVector2 new_dir = randomize_vector_angle(argos::CVector2::X);
    internal_event(ST_NEW_DIRECTION,
                   rcppsw::make_unique<new_direction_data>(new_dir.Angle()));
  }
  /*
   * No obstacles nearby and have not hit direction changing threshold--use the
   * diffusion vector only to set speeds.
   */
  argos::CVector2 vector;
  base_foraging_fsm::sensors()->calc_diffusion_vector(&vector);
  base_foraging_fsm::actuators()->set_heading(base_foraging_fsm::actuators()->max_wheel_speed() * vector);
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE(explore_fsm, new_direction, struct new_direction_data) {
  if (ST_NEW_DIRECTION != last_state()) {
    ER_DIAG("Executing ST_NEW_DIRECTION");
  }
  argos::CRadians current_dir = base_foraging_fsm::sensors()->calc_vector_to_light().Angle();

  /*
   * The new direction is only passed the first time this state is entered, so
   * save it. After that, a standard HFSM signal is passed we which ignore.
   */
  if (data) {
    m_new_dir = data->dir;
  }
  base_foraging_fsm::actuators()->set_heading(argos::CVector2(
      base_foraging_fsm::actuators()->max_wheel_speed() * 0.25, m_new_dir), true);

  /* We have changed direction and started a new exploration */
  if (std::fabs((current_dir - m_new_dir).GetValue()) < 0.1) {
    m_state.time_exploring_unsuccessfully = 0;
    internal_event(ST_EXPLORE);
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_ENTRY_DEFINE(explore_fsm, entry_explore, state_machine::no_event_data) {
  base_foraging_fsm::actuators()->leds_set_color(argos::CColor::MAGENTA);
}
HFSM_ENTRY_DEFINE(explore_fsm, entry_new_direction, state_machine::no_event_data) {
  base_foraging_fsm::actuators()->leds_set_color(argos::CColor::CYAN);
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void explore_fsm::init(void) {
  explore_time_reset();
  base_foraging_fsm::init();
} /* init() */

void explore_fsm::run(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* run() */


NS_END(fsm, fordyca);
