/**
 * @file random_foraging_fsm.cpp
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
#include "fordyca/fsm/random_foraging_fsm.hpp"
#include "fordyca/params/fsm_params.hpp"
#include "fordyca/controller/foraging_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
random_foraging_fsm::random_foraging_fsm(
    const struct params::fsm_params* params,
    std::shared_ptr<rcppsw::common::er_server> server,
    std::shared_ptr<controller::sensor_manager> sensors,
    std::shared_ptr<controller::actuator_manager> actuators) :
    base_foraging_fsm(server, sensors, actuators, ST_MAX_STATES),
    HFSM_CONSTRUCT_STATE(return_to_nest, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(leaving_nest, &start),
    entry_return_to_nest(),
    entry_leaving_nest(),
    HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(acquire_block, hfsm::top_state()),
    m_rng(argos::CRandom::CreateRNG("argos")),
    m_explore_fsm(params->times.unsuccessful_explore_dir_change,
                  server, sensors, actuators) {
  er_client::insmod("random_foraging_fsm",
         rcppsw::common::er_lvl::DIAG,
         rcppsw::common::er_lvl::NOM);
  hfsm::change_parent(ST_RETURN_TO_NEST, &start);
  hfsm::change_parent(ST_LEAVING_NEST, &start);
  m_explore_fsm.change_parent(explore_fsm::ST_EXPLORE, &acquire_block);
}

/*******************************************************************************
 * States
 ******************************************************************************/
HFSM_STATE_DEFINE(random_foraging_fsm, start, state_machine::event_data) {
  /* first time running FSM */
  if (state_machine::event_type::NORMAL == data->type()) {
    ER_NOM("Starting foraging");
    internal_event(ST_ACQUIRE_BLOCK);
    return controller::foraging_signal::HANDLED;
  }
  if (state_machine::event_type::CHILD == data->type()) {
    if (controller::foraging_signal::LEFT_NEST == data->signal()) {
      m_explore_fsm.init();
      internal_event(ST_ACQUIRE_BLOCK);
      return controller::foraging_signal::HANDLED;
    } else if (controller::foraging_signal::BLOCK_DROP == data->signal()) {
      internal_event(ST_LEAVING_NEST);
      return controller::foraging_signal::HANDLED;
    } else if (controller::foraging_signal::LEFT_NEST == data->signal()) {
      internal_event(ST_ACQUIRE_BLOCK);
      return controller::foraging_signal::HANDLED;
    }
  }
  return controller::foraging_signal::HANDLED;
}
HFSM_STATE_DEFINE(random_foraging_fsm, acquire_block, state_machine::event_data) {
  /*
   * All signals propagated up from the explore FSM are ignored; we only care
   * when the controller tells us we have actually picked up a block.
   *
   * BUGFIX 10/19/17: For some reason, you cannot call m_explore_fsm.run() from
   * this function IF the explore sub-FSM was what brought you to this function
   * in the first place (i.e. it does not play nice with recursion).
   */
  if (data && state_machine::event_type::CHILD == data->type()) {
    return controller::foraging_signal::HANDLED;
  } else if (data && controller::foraging_signal::BLOCK_PICKUP == data->signal()) {
    ER_NOM("Block acquired");
    internal_event(ST_RETURN_TO_NEST);
  }
  m_explore_fsm.run();
  return controller::foraging_signal::HANDLED;
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void random_foraging_fsm::init(void) {
  base_foraging_fsm::init();
  m_explore_fsm.init();
} /* init() */

bool random_foraging_fsm::is_exploring(void) const {
  return current_state() == ST_ACQUIRE_BLOCK && m_explore_fsm.is_searching();
} /* is_exploring() */

void random_foraging_fsm::run(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* run() */

NS_END(fsm, fordyca);
