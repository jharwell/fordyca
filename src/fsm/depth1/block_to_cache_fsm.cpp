/**
 * @file block_to_cache_fsm.cpp
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
#include "fordyca/fsm/depth1/block_to_cache_fsm.hpp"
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/depth1/foraging_sensors.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/params/fsm_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth1);
namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_to_cache_fsm::block_to_cache_fsm(
    const struct params::fsm_params* params,
    const std::shared_ptr<rcppsw::er::server>& server,
    const std::shared_ptr<controller::depth1::foraging_sensors>& sensors,
    const std::shared_ptr<controller::actuation_subsystem>& actuators,
    const std::shared_ptr<representation::perceived_arena_map>& map)
    : base_foraging_fsm(
          params->times.unsuccessful_explore_dir_change,
          server,
          std::static_pointer_cast<controller::base_sensing_subsystem>(sensors),
          actuators,
          ST_MAX_STATES),
      HFSM_CONSTRUCT_STATE(collision_avoidance, &start),
      entry_collision_avoidance(),
      entry_wait_for_signal(),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_free_block, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_block_pickup, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(transport_to_cache, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_cache_drop, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      m_pickup_count(0),
      m_sensors(sensors),
      m_block_fsm(params,
                  server,
                  std::static_pointer_cast<controller::depth0::foraging_sensors>(
                      sensors),
                  actuators,
                  map),
      m_cache_fsm(params, server, sensors, actuators, map),
      mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
                   HFSM_STATE_MAP_ENTRY_EX(&acquire_free_block),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_pickup,
                                               nullptr,
                                               &entry_wait_for_signal,
                                               nullptr),
                   HFSM_STATE_MAP_ENTRY_EX(&transport_to_cache),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_cache_drop,
                                               nullptr,
                                               &entry_wait_for_signal,
                                               nullptr),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&collision_avoidance,
                                               nullptr,
                                               &entry_collision_avoidance,
                                               nullptr),
                   HFSM_STATE_MAP_ENTRY_EX(&finished)} {
  insmod("block_to_cache_fsm", rcppsw::er::er_lvl::DIAG, rcppsw::er::er_lvl::NOM);
}

HFSM_STATE_DEFINE(block_to_cache_fsm, start, state_machine::event_data) {
  if (state_machine::event_type::NORMAL == data->type()) {
    if (controller::foraging_signal::ACQUIRE_FREE_BLOCK == data->signal()) {
      internal_event(ST_ACQUIRE_FREE_BLOCK);
      return controller::foraging_signal::HANDLED;
    }
  } else {
    ER_FATAL_SENTINEL("FATAL: Cannot handle child signals");
  }
  return controller::foraging_signal::HANDLED;
}
HFSM_STATE_DEFINE_ND(block_to_cache_fsm, acquire_free_block) {
  if (m_block_fsm.task_finished()) {
    internal_event(ST_WAIT_FOR_BLOCK_PICKUP);
  } else {
    m_block_fsm.task_execute();
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE_ND(block_to_cache_fsm, transport_to_cache) {
  if (m_cache_fsm.task_finished()) {
    m_cache_fsm.task_reset();
    actuators()->differential_drive()->stop_wheels();
    internal_event(ST_WAIT_FOR_CACHE_DROP);
  } else {
    m_cache_fsm.task_execute();
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE(block_to_cache_fsm,
                  wait_for_block_pickup,
                  state_machine::event_data) {
  if (controller::foraging_signal::BLOCK_PICKUP == data->signal()) {
    ER_DIAG("Block pickup signal received");
    m_block_fsm.task_reset();
    m_pickup_count = 0;
    internal_event(ST_TRANSPORT_TO_CACHE);
  }
  /*
   * It is possible that robots can be waiting in this wait indefinitely for a
   * block pickup signal that will never come if they got here by "detecting" a
   * block by sprawling across multiple blocks (i.e. all ground sensors did not
   * detect the same block).
   *
   * In that case, the timeout here will cause the robot to try again, and
   * because of the decaying relevance of cells, it will eventually pick a
   * different block than the one that got it into this predicament, and the
   * system will be able to continue profitably.
   */
  ++m_pickup_count;
  if (m_pickup_count >= kPICKUP_TIMEOUT) {
    m_pickup_count = 0;
    m_block_fsm.task_reset();
    internal_event(ST_ACQUIRE_FREE_BLOCK);
  }

  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE(block_to_cache_fsm,
                  wait_for_cache_drop,
                  state_machine::event_data) {
  if (controller::foraging_signal::BLOCK_DROP == data->signal()) {
    ER_DIAG("Block drop signal received");
    internal_event(ST_FINISHED);
  } else if (controller::foraging_signal::CACHE_VANISHED == data->signal()) {
    m_cache_fsm.task_reset();
    internal_event(ST_TRANSPORT_TO_CACHE);
  }
  return controller::foraging_signal::HANDLED;
}

__const HFSM_STATE_DEFINE_ND(block_to_cache_fsm, finished) {
  return controller::foraging_signal::HANDLED;
}

/*******************************************************************************
 * Base Diagnostics
 ******************************************************************************/
__pure bool block_to_cache_fsm::is_exploring_for_block(void) const {
  return m_block_fsm.is_exploring_for_block();
} /* is_exploring_for_block() */

__pure bool block_to_cache_fsm::is_avoiding_collision(void) const {
  return m_block_fsm.is_avoiding_collision() ||
         m_cache_fsm.is_avoiding_collision();
} /* is_avoiding_collision() */

/*******************************************************************************
 * Depth0 Diagnostics
 ******************************************************************************/
__pure bool block_to_cache_fsm::is_acquiring_block(void) const {
  return (current_state() == ST_ACQUIRE_FREE_BLOCK) &&
         m_block_fsm.is_acquiring_block();
} /* is_acquiring_block */

__pure bool block_to_cache_fsm::is_vectoring_to_block(void) const {
  return (current_state() == ST_ACQUIRE_FREE_BLOCK) &&
         m_block_fsm.is_vectoring_to_block();
} /* is_vectoring_to_block */

/*******************************************************************************
 * Depth1 Diagnostics
 ******************************************************************************/
__pure bool block_to_cache_fsm::is_exploring_for_cache(void) const {
  return is_transporting_to_cache() && m_cache_fsm.is_exploring_for_cache();
} /* is_exploring_for_cache */

__pure bool block_to_cache_fsm::is_vectoring_to_cache(void) const {
  return is_transporting_to_cache() && m_cache_fsm.is_vectoring_to_cache();
} /* is_vectoring_to_cache */

__pure bool block_to_cache_fsm::is_acquiring_cache(void) const {
  return is_transporting_to_cache() && m_cache_fsm.is_acquiring_cache();
} /* is_acquiring_cache */

__pure bool block_to_cache_fsm::is_transporting_to_cache(void) const {
  return current_state() == ST_TRANSPORT_TO_CACHE;
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void block_to_cache_fsm::init(void) {
  base_foraging_fsm::init();
  m_cache_fsm.task_reset();
  m_block_fsm.task_reset();
} /* init() */

bool block_to_cache_fsm::cache_acquired(void) const {
  return current_state() == ST_WAIT_FOR_CACHE_DROP;
} /* cache_acquired() */

bool block_to_cache_fsm::block_acquired(void) const {
  return current_state() == ST_WAIT_FOR_BLOCK_PICKUP;
} /* block_acquired() */

void block_to_cache_fsm::task_start(
    const rcppsw::task_allocation::taskable_argument* const arg) {
  auto* a = dynamic_cast<const tasks::foraging_signal_argument* const>(arg);
  ER_ASSERT(a, "FATAL: bad argument passed");
  inject_event(a->signal(), state_machine::event_type::NORMAL);

  inject_event(controller::foraging_signal::ACQUIRE_FREE_BLOCK,
               state_machine::event_type::NORMAL);
}

void block_to_cache_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* task_execute() */

NS_END(depth1, fsm, fordyca);
