/**
 * @file block_to_nest_fsm.cpp
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
#include "fordyca/fsm/block_to_nest_fsm.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/controller/depth1/foraging_sensors.hpp"
#include "fordyca/controller/actuator_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_to_nest_fsm::block_to_nest_fsm(
    const struct params::fsm_params* params,
    const std::shared_ptr<rcppsw::er::server>& server,
    const std::shared_ptr<controller::depth1::foraging_sensors>& sensors,
    const std::shared_ptr<controller::actuator_manager>& actuators,
    const std::shared_ptr<const representation::perceived_arena_map>& map) :
    base_foraging_fsm(server,
                      std::static_pointer_cast<controller::base_foraging_sensors>(sensors),
                      actuators, ST_MAX_STATES),
    HFSM_CONSTRUCT_STATE(transport_to_nest, &start),
    HFSM_CONSTRUCT_STATE(collision_avoidance, &start),
    entry_transport_to_nest(),
    entry_collision_avoidance(),
    HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(acquire_free_block, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(wait_for_block_pickup, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(acquire_cached_block, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(wait_for_cache_pickup, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
    entry_wait_for_pickup(),
    m_pickup_count(0),
    m_sensors(sensors),
    m_block_fsm(params, server, sensors, actuators, map),
    m_cache_fsm(params, server, sensors, actuators, map),
    mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
        HFSM_STATE_MAP_ENTRY_EX(&acquire_free_block),
      HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_pickup, NULL,
                                  &entry_wait_for_pickup, NULL),
        HFSM_STATE_MAP_ENTRY_EX(&acquire_cached_block),
      HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_cache_pickup, NULL,
                                  &entry_wait_for_pickup, NULL),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&transport_to_nest, NULL,
                                    &entry_transport_to_nest, NULL),
      HFSM_STATE_MAP_ENTRY_EX_ALL(&collision_avoidance, NULL,
                                  &entry_collision_avoidance, NULL),
      HFSM_STATE_MAP_ENTRY_EX(&finished)} {}

HFSM_STATE_DEFINE(block_to_nest_fsm, start, state_machine::event_data) {
  if (state_machine::event_type::NORMAL == data->type()) {
    if (controller::foraging_signal::ACQUIRE_FREE_BLOCK == data->signal()) {
      internal_event(ST_ACQUIRE_FREE_BLOCK);
      return controller::foraging_signal::HANDLED;
    } else if (controller::foraging_signal::ACQUIRE_CACHED_BLOCK == data->signal()) {
      internal_event(ST_ACQUIRE_CACHED_BLOCK);
      return controller::foraging_signal::HANDLED;
    }
  } else if (state_machine::event_type::CHILD == data->type()) {
    if (controller::foraging_signal::BLOCK_DROP == data->signal()) {
      internal_event(ST_FINISHED);
      return controller::foraging_signal::HANDLED;
    } else if (controller::foraging_signal::COLLISION_IMMINENT == data->signal()) {
      internal_event(ST_COLLISION_AVOIDANCE);
      return controller::foraging_signal::HANDLED;
    }
  }
  return controller::foraging_signal::HANDLED;
}
HFSM_STATE_DEFINE_ND(block_to_nest_fsm, acquire_free_block) {
  if (m_block_fsm.task_finished()) {
    internal_event(ST_WAIT_FOR_BLOCK_PICKUP);
  } else {
    m_block_fsm.task_execute();
  }
  return controller::foraging_signal::HANDLED;
}
HFSM_STATE_DEFINE_ND(block_to_nest_fsm, acquire_cached_block) {
  if (m_cache_fsm.task_finished()) {
    internal_event(ST_WAIT_FOR_CACHE_PICKUP);
  } else {
    m_cache_fsm.task_execute();
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE(block_to_nest_fsm, wait_for_block_pickup, state_machine::event_data) {
  if (controller::foraging_signal::BLOCK_PICKUP == data->signal()) {
    m_block_fsm.task_reset();
    m_pickup_count = 0;
    internal_event(ST_TRANSPORT_TO_NEST);
  }
  /*
   * It is possible that robots can be waiting in this wait indefinitely for a
   * block pickup signal that will never come if they got here by "detecting" a
   * block by sprawling across multiple blocks (i.e. all ground sensors did not
   * detect the same block). This is only a problem for generalists, who
   * cannot/do not abort their tasks.
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

HFSM_STATE_DEFINE(block_to_nest_fsm, wait_for_cache_pickup, state_machine::event_data) {
  if (controller::foraging_signal::BLOCK_PICKUP == data->signal()) {
    m_cache_fsm.task_reset();
    internal_event(ST_TRANSPORT_TO_NEST);
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_ENTRY_DEFINE_ND(block_to_nest_fsm, entry_wait_for_pickup) {
  base_foraging_fsm::actuators()->leds_set_color(argos::CColor::WHITE);
}

__const HFSM_STATE_DEFINE_ND(block_to_nest_fsm, finished) {
  return controller::foraging_signal::HANDLED;
}

/*******************************************************************************
 * Base Diagnostics
 ******************************************************************************/
__pure bool block_to_nest_fsm::is_exploring_for_block(void) const {
  return m_block_fsm.is_exploring_for_block();
} /* is_exploring_for_block() */

__pure bool block_to_nest_fsm::is_avoiding_collision(void) const {
  return m_block_fsm.is_avoiding_collision() ||
      m_cache_fsm.is_avoiding_collision();
} /* is_avoiding_collision() */

__pure bool block_to_nest_fsm::is_transporting_to_nest(void) const {
  return current_state() == ST_TRANSPORT_TO_NEST;
} /* is_transporting_to_nest() */

/*******************************************************************************
 * Depth0 Diagnostics
 ******************************************************************************/
__pure bool block_to_nest_fsm::is_acquiring_block(void) const {
  return m_block_fsm.is_acquiring_block();
} /* is_acquiring_block() */

__pure bool block_to_nest_fsm::is_vectoring_to_block(void) const {
  return m_block_fsm.is_vectoring_to_block();
} /* is_vectoring_to_block() */

/*******************************************************************************
 * Depth1 Diagnostics
 ******************************************************************************/
__pure bool block_to_nest_fsm::is_exploring_for_cache(void) const {
  return m_cache_fsm.is_exploring_for_cache();
} /* is_exploring_for_cache() */

__pure bool block_to_nest_fsm::is_vectoring_to_cache(void) const {
  return m_cache_fsm.is_vectoring_to_cache();
} /* is_vectoring_to_cache() */

__pure bool block_to_nest_fsm::is_acquiring_cache(void) const {
  return m_cache_fsm.is_acquiring_cache();
} /* is_acquiring_cache() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
bool block_to_nest_fsm::cache_acquired(void) const {
  return current_state() == ST_WAIT_FOR_CACHE_PICKUP;
} /* cache_acquired() */

bool block_to_nest_fsm::block_acquired(void) const {
  return current_state() == ST_WAIT_FOR_BLOCK_PICKUP;
} /* block_acquired() */

void block_to_nest_fsm::init(void) {
  base_foraging_fsm::init();
  m_cache_fsm.task_reset();
  m_block_fsm.task_reset();
} /* init() */

void block_to_nest_fsm::task_start(const rcppsw::task_allocation::taskable_argument* const arg) {
  const tasks::foraging_signal_argument* const a =
      dynamic_cast<const tasks::foraging_signal_argument* const>(arg);
  ER_ASSERT(a, "FATAL: bad argument passed");
  inject_event(a->signal(), state_machine::event_type::NORMAL);
}

void block_to_nest_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* task_execute() */

NS_END(fsm, fordyca);
