/**
 * @file stateless_foraging_fsm.cpp
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
#include "fordyca/fsm/depth0/stateless_foraging_fsm.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/params/fsm_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth0);
namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
stateless_foraging_fsm::stateless_foraging_fsm(
    const struct params::fsm_params *params,
    const std::shared_ptr<rcppsw::er::server> &server,
    const std::shared_ptr<controller::base_foraging_sensors>& sensors,
    const std::shared_ptr<controller::actuator_manager>& actuators)
    : base_foraging_fsm(params->times.unsuccessful_explore_dir_change,
                        server, sensors, actuators, ST_MAX_STATES),
      HFSM_CONSTRUCT_STATE(transport_to_nest, &start),
      HFSM_CONSTRUCT_STATE(leaving_nest, &start),
      HFSM_CONSTRUCT_STATE(collision_avoidance, &start),
      entry_transport_to_nest(),
      entry_leaving_nest(),
      entry_collision_avoidance(),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_block, hfsm::top_state()),
      m_rng(argos::CRandom::CreateRNG("argos")),
      m_explore_fsm(params->times.unsuccessful_explore_dir_change,
                    server,
                    sensors,
                    actuators),
      mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
                   HFSM_STATE_MAP_ENTRY_EX(&acquire_block),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&transport_to_nest,
                                               NULL,
                                               &entry_transport_to_nest,
                                               NULL),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&leaving_nest,
                                               NULL,
                                               &entry_leaving_nest,
                                               NULL),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&collision_avoidance,
                                               NULL,
                                               &entry_collision_avoidance,
                                               NULL)} {
  client::insmod("stateless_foraging_fsm",
                 rcppsw::er::er_lvl::DIAG,
                 rcppsw::er::er_lvl::NOM);
}

/*******************************************************************************
 * States
 ******************************************************************************/
HFSM_STATE_DEFINE(stateless_foraging_fsm, start, state_machine::event_data) {
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
    } else if (controller::foraging_signal::COLLISION_IMMINENT ==
               data->signal()) {
      internal_event(ST_COLLISION_AVOIDANCE);
      return controller::foraging_signal::HANDLED;
    }
  }
  ER_FATAL_SENTINEL("FATAL: Unhandled signal");
  return controller::foraging_signal::HANDLED;
}
HFSM_STATE_DEFINE(stateless_foraging_fsm,
                  acquire_block,
                  state_machine::event_data) {
  /*
   * Wait in the finished state until the controller tells us we have dropped a
   * block.
   */
  if (m_explore_fsm.task_finished()) {
    if (nullptr != data &&
        controller::foraging_signal::BLOCK_PICKUP == data->signal()) {
      m_explore_fsm.task_reset();
      ER_NOM("Block acquired");
      internal_event(ST_TRANSPORT_TO_NEST);
    }
  }

  m_explore_fsm.task_execute();
  return controller::foraging_signal::HANDLED;
}

/*******************************************************************************
 * Base Diagnostics
 ******************************************************************************/
bool stateless_foraging_fsm::is_exploring_for_block(void) const {
  return current_state() == ST_ACQUIRE_BLOCK;
} /* is_exploring_for_block() */

__pure bool stateless_foraging_fsm::is_avoiding_collision(void) const {
  return m_explore_fsm.is_avoiding_collision();
} /* is_avoiding_collision() */

bool stateless_foraging_fsm::is_transporting_to_nest(void) const {
  return current_state() == ST_TRANSPORT_TO_NEST;
} /* is_transporting_to_nest() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void stateless_foraging_fsm::init(void) {
  base_foraging_fsm::init();
  m_explore_fsm.init();
} /* init() */

void stateless_foraging_fsm::run(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* run() */

NS_END(depth0, fsm, fordyca);
