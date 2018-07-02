/**
 * @file cache_transferer_fsm.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/fsm/depth2/cache_transferer_fsm.hpp"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/datatypes/color.h>

#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/depth0/block_selector.hpp"
#include "fordyca/controller/depth0/sensing_subsystem.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/params/fsm_params.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth2);
namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
cache_transferer_fsm::cache_transferer_fsm(
    const struct params::fsm_params* params,
    const std::shared_ptr<rcppsw::er::server>& server,
    controller::saa_subsystem* const saa,
    representation::perceived_arena_map* const map)
    : base_foraging_fsm(server, saa, ST_MAX_STATES),
      entry_wait_for_signal(),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_src_cache, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_block_pickup, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_dest_cache, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_block_drop, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      m_cache_fsm(params, server, saa, map),
      mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
                   HFSM_STATE_MAP_ENTRY_EX(&acquire_src_cache),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_pickup,
                                               nullptr,
                                               &entry_wait_for_signal,
                                               nullptr),
                   HFSM_STATE_MAP_ENTRY_EX(&acquire_dest_cache),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_drop,
                                               nullptr,
                                               &entry_wait_for_signal,
                                               nullptr),
                   HFSM_STATE_MAP_ENTRY_EX(&finished)} {
  client::insmod("cache_transferer_fsm",
                 rcppsw::er::er_lvl::DIAG,
                 rcppsw::er::er_lvl::NOM);
}

HFSM_STATE_DEFINE_ND(cache_transferer_fsm, start) {
  internal_event(ST_ACQUIRE_SRC_CACHE);
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE_ND(cache_transferer_fsm, acquire_src_cache) {
  ER_DIAG("Executing ST_ACQUIRE_SRC_CACHE");
  if (m_cache_fsm.task_finished()) {
    actuators()->differential_drive().stop();
    internal_event(ST_WAIT_FOR_BLOCK_PICKUP);
  } else {
    m_cache_fsm.task_execute();
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE(cache_transferer_fsm,
                  wait_for_block_pickup,
                  state_machine::event_data) {
  if (controller::foraging_signal::BLOCK_PICKUP == data->signal()) {
    m_cache_fsm.task_reset();
    internal_event(ST_ACQUIRE_DEST_CACHE);
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE_ND(cache_transferer_fsm, acquire_dest_cache) {
  ER_DIAG("Executing ST_ACQUIRE_DEST_CACHE");
  if (m_cache_fsm.task_finished()) {
    actuators()->differential_drive().stop();
    internal_event(ST_WAIT_FOR_BLOCK_DROP);
  } else {
    m_cache_fsm.task_execute();
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE(cache_transferer_fsm,
                  wait_for_block_drop,
                  state_machine::event_data) {
  if (controller::foraging_signal::BLOCK_DROP == data->signal()) {
    m_cache_fsm.task_reset();
    internal_event(ST_FINISHED);
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE_ND(cache_transferer_fsm, finished) {
  if (ST_FINISHED != last_state()) {
    ER_DIAG("Executing ST_FINISHED");
  }
  return controller::foraging_signal::HANDLED;
}

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
FSM_WRAPPER_DEFINE(bool,
                   cache_transferer_fsm,
                   is_avoiding_collision,
                   m_cache_fsm);

FSM_WRAPPER_DEFINE(bool, cache_transferer_fsm, goal_acquired, m_cache_fsm);

FSM_WRAPPER_DEFINE(bool, cache_transferer_fsm, is_vectoring_to_goal, m_cache_fsm);

FSM_WRAPPER_DEFINE(bool,
                   cache_transferer_fsm,
                   is_exploring_for_goal,
                   m_cache_fsm);

FSM_WRAPPER_DEFINE(acquisition_goal_type,
                   cache_transferer_fsm,
                   acquisition_goal,
                   m_cache_fsm);

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void cache_transferer_fsm::init(void) {
  base_foraging_fsm::init();
  m_cache_fsm.task_reset();
} /* init() */

void cache_transferer_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* task_execute() */

NS_END(depth2, fsm, fordyca);
