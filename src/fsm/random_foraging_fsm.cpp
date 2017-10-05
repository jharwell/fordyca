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
  insmod("random_foraging_fsm",
         rcppsw::common::er_lvl::DIAG,
         rcppsw::common::er_lvl::NOM);
  hfsm::change_parent(ST_RETURN_TO_NEST, &start);
  hfsm::change_parent(ST_LEAVING_NEST, &start);
  m_explore_fsm.change_parent(explore_fsm::ST_EXPLORE, &acquire_block);
}

/*******************************************************************************
 * States
 ******************************************************************************/
__noreturn HFSM_STATE_DEFINE(random_foraging_fsm, start, state_machine::no_event_data) {
  /* first time running FSM */
  if (nullptr == data) {
    internal_event(ST_ACQUIRE_BLOCK);
  }

  if (state_machine::event_type::CHILD == data->type()) {
    if (controller::foraging_signal::LEFT_NEST == data->signal()) {
      internal_event(ST_ACQUIRE_BLOCK);
    } else if (controller::foraging_signal::ARRIVED_IN_NEST == data->signal()) {
      internal_event(ST_LEAVING_NEST);
    }
  }
  ER_ASSERT(0, "FATAL: Unhandled signal");
}
HFSM_STATE_DEFINE(random_foraging_fsm, acquire_block, state_machine::event_data) {
  if (state_machine::event_type::CHILD == data->type() &&
      controller::foraging_signal::BLOCK_ACQUIRED == data->signal()) {
      internal_event(ST_RETURN_TO_NEST);
  }
  /*
   * BLOCK_LOCATED signal ignored from explore FSM; we only care when the
   * controller tells us we have actually picked up a block
   */
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

NS_END(fsm, fordyca);
