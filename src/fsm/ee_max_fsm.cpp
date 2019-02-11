/**
 * @file ee_max_fsm.cpp
 *
 * @copyright 2019 Anthony Chen/John Harwell, All rights reserved.
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
#include "fordyca/fsm/ee_max_fsm.hpp"
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/controller/random_explore_behavior.hpp"
#include "fordyca/controller/energy_supervisor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
namespace state_machine = rcppsw::patterns::state_machine;
namespace ta = rcppsw::task_allocation;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

// get pointer to taskable object (crw_fsm)
ee_max_fsm::ee_max_fsm(const controller::ee_decision_matrix* matrix,
                        controller::saa_subsystem* const saa)
    : base_foraging_fsm(saa, ST_MAX_STATES),
      mc_matrix(matrix),
      saa(saa),
      ER_CLIENT_INIT("fordyca.fsm.depth0.ee_max"),
      HFSM_CONSTRUCT_STATE(transport_to_nest, &start),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(foraging, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(charging, hfsm::top_state()),
      mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
                   HFSM_STATE_MAP_ENTRY_EX(&foraging),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&transport_to_nest,
                                               nullptr,
                                               &entry_transport_to_nest,
                                               nullptr),
                   HFSM_STATE_MAP_ENTRY_EX(&charging)} {}

/*******************************************************************************
 * States
 ******************************************************************************/
HFSM_STATE_DEFINE(ee_max_fsm, start, state_machine::event_data) {
  /* first time running FSM */
  if (state_machine::event_type::NORMAL == data->type()) {
    internal_event(ST_FORAGING);
    return controller::foraging_signal::HANDLED;
  }
  if (state_machine::event_type::CHILD == data->type()) {
    if (controller::foraging_signal::LEFT_NEST == data->signal()) {
      internal_event(ST_FORAGING);
      return controller::foraging_signal::HANDLED;
    } else if (controller::foraging_signal::ENTERED_NEST == data->signal()) {
      internal_event(ST_CHARGING);
      return controller::foraging_signal::HANDLED;
    }
  }
  ER_FATAL_SENTINEL("Unhandled signal");
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE(ee_max_fsm, foraging, state_machine::event_data) {
  if (state_machine::event_type::NORMAL == data->type()) {
    if (controller::foraging_signal::BLOCK_DROP == data->signal()) {
      ER_INFO("HIT SIGNAL RECEPTION");
      internal_event(ST_CHARGING);
      return controller::foraging_signal::HANDLED;
    }
  }
  controller::energy_supervisor selector(mc_matrix);
  float low_energy = selector.getLowerThres();
  double current_energy = saa->sensing()->battery().readings().available_charge;
  ER_INFO("FSM:\tRobot is Foraging........ENERGY: %f", current_energy);
  if (current_energy <= low_energy) {
    ER_INFO("FSM:\tRobot enters Retreating from Foraging State");
    internal_event(ST_RETREATING);
  } else {
    if(taskable_fsm->task_finished()) {
      taskable_fsm->task_reset();
      taskable_fsm->task_start(nullptr);
    }


    taskable_fsm->task_execute();
  }

}

HFSM_STATE_DEFINE_ND(ee_max_fsm, charging) {
  saa->actuation()->differential_drive().stop();
  controller::energy_supervisor selector(mc_matrix);
  float charged_energy = selector.getHigherThres();
  double current_energy = saa->sensing()->battery().readings().available_charge;
  ER_INFO("FSM:\tRobot is Charging........CHARGE: %f", current_energy);
  if (current_energy == charged_energy) {
    ER_INFO("FSM:\tRobot enters Foraging from Charging State");
    internal_event(ST_FORAGING);
  }
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/

 void ee_max_fsm::run(void) {
   inject_event(controller::foraging_signal::FSM_RUN,
                state_machine::event_type::NORMAL);
 } /* run() */

NS_END(fsm, fordyca);
