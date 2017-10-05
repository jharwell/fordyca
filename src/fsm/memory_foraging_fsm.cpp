/**
 * @file memory_foraging_fsm.cpp
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
#include "fordyca/fsm/memory_foraging_fsm.hpp"
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include "fordyca/controller/block_selector.hpp"
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
memory_foraging_fsm::memory_foraging_fsm(
    const struct params::fsm_params* params,
    const std::shared_ptr<rcppsw::common::er_server>& server,
    const std::shared_ptr<controller::sensor_manager>& sensors,
    const std::shared_ptr<controller::actuator_manager>& actuators,
    const std::shared_ptr<const representation::perceived_arena_map>& map) :
    base_foraging_fsm(server, sensors, actuators, ST_MAX_STATES),
    HFSM_CONSTRUCT_STATE(return_to_nest, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(leaving_nest, hfsm::top_state()),
    entry_return_to_nest(),
    entry_leaving_nest(),
    HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(acquire_free_block, hfsm::top_state()),
    exit_acquire_free_block(),
    mc_nest_center(params->nest_center),
    m_rng(argos::CRandom::CreateRNG("argos")),
    m_vector_fsm(params->times.frequent_collision_thresh,
                 server, sensors, actuators),
    m_block_fsm(params, server, sensors, actuators, map) {
  hfsm::change_parent(ST_RETURN_TO_NEST, &start);
  hfsm::change_parent(ST_LEAVING_NEST, &start);
    }

HFSM_STATE_DEFINE(memory_foraging_fsm, start, state_machine::event_data) {
  if (state_machine::event_type::NORMAL == data->type()) {
    internal_event(ST_ACQUIRE_FREE_BLOCK);
  } else if (state_machine::event_type::CHILD == data->type()) {
    if (controller::foraging_signal::LEFT_NEST == data->signal()) {
      internal_event(ST_ACQUIRE_FREE_BLOCK);
    } else if (controller::foraging_signal::ARRIVED_IN_NEST == data->signal()) {
      internal_event(ST_LEAVING_NEST);
    }
  }
  return state_machine::event_signal::HANDLED;
}
HFSM_STATE_DEFINE(memory_foraging_fsm, acquire_free_block, state_machine::event_data) {
  if (m_block_fsm.task_finished()) {
    internal_event(ST_RETURN_TO_NEST);
  }
  m_block_fsm.task_execute();
    return state_machine::event_signal::HANDLED;
}

HFSM_EXIT_DEFINE(memory_foraging_fsm, exit_acquire_free_block) {
  m_block_fsm.task_reset();
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void memory_foraging_fsm::init(void) {
  base_foraging_fsm::init();
  m_block_fsm.task_reset();
} /* init() */

void memory_foraging_fsm::run(void) {
  inject_event(state_machine::event_signal::IGNORED,
               state_machine::event_type::NORMAL);
} /* run() */


NS_END(controller, fordyca);
