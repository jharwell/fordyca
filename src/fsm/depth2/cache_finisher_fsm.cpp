/**
 * @file cache_finisher_fsm.cpp
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
#include "fordyca/fsm/depth2/cache_finisher_fsm.hpp"
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
cache_finisher_fsm::cache_finisher_fsm(
    const struct params::fsm_params* params,
    const std::shared_ptr<rcppsw::er::server>& server,
    const std::shared_ptr<controller::saa_subsystem>& saa,
    const std::shared_ptr<representation::perceived_arena_map>& map)
    : base_foraging_fsm(server, saa, ST_MAX_STATES),
      entry_wait_for_signal(),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_block, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_block_pickup, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_new_cache, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_block_drop, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      m_block_fsm(params, server, saa, map),
      m_cache_fsm(params, server, saa, map),
      mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
                   HFSM_STATE_MAP_ENTRY_EX(&acquire_block),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_pickup,
                                               nullptr,
                                               &entry_wait_for_signal,
                                               nullptr),
                   HFSM_STATE_MAP_ENTRY_EX(&acquire_new_cache),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_drop,
                                               nullptr,
                                               &entry_wait_for_signal,
                                               nullptr),
                   HFSM_STATE_MAP_ENTRY_EX(&finished)} {
  client::insmod("cache_finisher_fsm",
                 rcppsw::er::er_lvl::DIAG,
                 rcppsw::er::er_lvl::NOM);
}

HFSM_STATE_DEFINE_ND(cache_finisher_fsm, start) {
  internal_event(ST_ACQUIRE_BLOCK);
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE_ND(cache_finisher_fsm, acquire_block) {
  ER_DIAG("Executing ST_ACQUIRE_BLOCK");
  if (m_block_fsm.task_finished()) {
    actuators()->differential_drive().stop();
    internal_event(ST_WAIT_FOR_BLOCK_PICKUP);
  } else {
    m_block_fsm.task_execute();
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE(cache_finisher_fsm,
                  wait_for_block_pickup,
                  state_machine::event_data) {
  if (controller::foraging_signal::BLOCK_PICKUP == data->signal()) {
    m_block_fsm.task_reset();
    m_pickup_count = 0;
    internal_event(ST_ACQUIRE_NEW_CACHE);
  }
  ++m_pickup_count;
  if (m_pickup_count >= kPICKUP_TIMEOUT) {
    m_pickup_count = 0;
    m_block_fsm.task_reset();
    internal_event(ST_ACQUIRE_BLOCK);
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE_ND(cache_finisher_fsm, acquire_new_cache) {
  ER_DIAG("Executing ST_ACQUIRE_NEW_CACHE");
  if (m_cache_fsm.task_finished()) {
    actuators()->differential_drive().stop();
    internal_event(ST_WAIT_FOR_BLOCK_DROP);
  } else {
    m_cache_fsm.task_execute();
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE(cache_finisher_fsm,
                  wait_for_block_drop,
                  state_machine::event_data) {
  if (controller::foraging_signal::BLOCK_DROP == data->signal()) {
    m_cache_fsm.task_reset();
    internal_event(ST_FINISHED);
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE_ND(cache_finisher_fsm, finished) {
  if (ST_FINISHED != last_state()) {
    ER_DIAG("Executing ST_FINISHED");
  }
  return controller::foraging_signal::HANDLED;
}

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
__pure bool cache_finisher_fsm::is_avoiding_collision(void) const {
  return m_block_fsm.is_avoiding_collision() ||
         m_cache_fsm.is_avoiding_collision();
} /* is_avoiding_collision() */

FSM_WRAPPER_DEFINE(bool, cache_finisher_fsm,
                   goal_acquired,
                   m_block_fsm);

FSM_WRAPPER_DEFINE(bool, cache_finisher_fsm,
                   is_vectoring_to_goal,
                   m_block_fsm);

FSM_WRAPPER_DEFINE(bool, cache_finisher_fsm,
                   is_exploring_for_goal,
                   m_block_fsm);

acquisition_goal_type cache_finisher_fsm::acquisition_goal(void) const {
  if (m_block_fsm.task_running() ||
      current_state() == ST_WAIT_FOR_BLOCK_PICKUP) {
    return m_block_fsm.acquisition_goal();
  } else if (m_cache_fsm.task_running() ||
             current_state() == ST_WAIT_FOR_BLOCK_DROP) {
    return m_cache_fsm.acquisition_goal();
  }
  return goal_type::kNone;
} /* acquisition_goal() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void cache_finisher_fsm::init(void) {
  base_foraging_fsm::init();
  m_block_fsm.task_reset();
  m_cache_fsm.task_reset();
} /* init() */

void cache_finisher_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* task_execute() */

NS_END(depth2, fsm, fordyca);
