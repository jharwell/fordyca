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
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include "fordyca/params/fsm_params.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/controller/base_foraging_sensors.hpp"
#include "fordyca/controller/actuator_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_explore_fsm::base_explore_fsm(
    double unsuccessful_dir_change_thresh,
    const std::shared_ptr<rcppsw::er::server>& server,
    const std::shared_ptr<controller::base_foraging_sensors>& sensors,
    const std::shared_ptr<controller::actuator_manager>& actuators,
                                   uint8_t max_states) :
    base_foraging_fsm(server, sensors, actuators, max_states),
    HFSM_CONSTRUCT_STATE(new_direction, hfsm::top_state()),
    entry_new_direction(),
    entry_explore(),
    mc_dir_change_thresh(unsuccessful_dir_change_thresh),
    m_rng(argos::CRandom::CreateRNG("argos")),
    m_state(),
    m_new_dir() {
  insmod("base_explore_fsm",
         rcppsw::er::er_lvl::DIAG,
         rcppsw::er::er_lvl::NOM);
}

HFSM_STATE_DEFINE(base_explore_fsm, new_direction, state_machine::event_data) {
  argos::CRadians current_dir = base_foraging_fsm::sensors()->calc_vector_to_light().Angle();

  /*
   * The new direction is only passed the first time this state is entered, so
   * save it. After that, a standard HFSM signal is passed we which ignore.
   */
  const new_direction_data* dir_data = dynamic_cast<const new_direction_data*>(data);
  if (dir_data) {
    m_new_dir = dir_data->dir;
  }
  base_foraging_fsm::actuators()->set_heading(argos::CVector2(
      base_foraging_fsm::actuators()->max_wheel_speed() * 0.25, m_new_dir), true);

  /* We have changed direction and started a new exploration */
  if (std::fabs((current_dir - m_new_dir).GetValue()) < 0.1) {
    m_state.time_exploring_unsuccessfully = 0;
    internal_event(previous_state());
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_ENTRY_DEFINE_ND(base_explore_fsm, entry_explore) {
  base_foraging_fsm::actuators()->leds_set_color(argos::CColor::MAGENTA);
}
HFSM_ENTRY_DEFINE_ND(base_explore_fsm, entry_new_direction) {
  base_foraging_fsm::actuators()->leds_set_color(argos::CColor::CYAN);
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void base_explore_fsm::init(void) {
  explore_time_reset();
  base_foraging_fsm::init();
} /* init() */

void base_explore_fsm::run(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* run() */


NS_END(fsm, fordyca);
