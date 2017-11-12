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
#include "fordyca/controller/depth1/foraging_sensors.hpp"
#include "fordyca/controller/foraging_signal.hpp"

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
    const std::shared_ptr<rcppsw::common::er_server>& server,
    const std::shared_ptr<controller::depth1::foraging_sensors>& sensors,
    const std::shared_ptr<controller::actuator_manager>& actuators,
    const std::shared_ptr<const representation::perceived_arena_map>& map) :
    base_foraging_fsm(server,
                      std::static_pointer_cast<controller::base_foraging_sensors>(sensors),
                      actuators, ST_MAX_STATES),
    HFSM_CONSTRUCT_STATE(collision_avoidance, &start),
    entry_collision_avoidance(),
    HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(acquire_free_block, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(transport_to_cache, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
    m_sensors(sensors),
    m_block_fsm(params, server,
                std::static_pointer_cast<controller::depth0::foraging_sensors>(sensors),
                actuators, map),
    m_cache_fsm(params, server, sensors, actuators, map),
    mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
      HFSM_STATE_MAP_ENTRY_EX(&acquire_free_block),
      HFSM_STATE_MAP_ENTRY_EX(&transport_to_cache),
      HFSM_STATE_MAP_ENTRY_EX_ALL(&collision_avoidance, NULL,
                                  &entry_collision_avoidance, NULL),
      HFSM_STATE_MAP_ENTRY_EX(&finished)} {}

HFSM_STATE_DEFINE(block_to_cache_fsm, start, state_machine::event_data) {
  if (state_machine::event_type::NORMAL == data->type()) {
    if (controller::foraging_signal::ACQUIRE_FREE_BLOCK == data->signal()) {
      internal_event(ST_ACQUIRE_FREE_BLOCK);
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
HFSM_STATE_DEFINE(block_to_cache_fsm, acquire_free_block, state_machine::event_data) {
  ER_ASSERT(state_machine::event_type::NORMAL == data->type(), "Bad event type");

  /*
   * We may get a BLOCK_PICKUP signal even if we have not yet finished vectoring
   * to our chosen block, if there are a bunch of blocks really close
   * together; the simulation will signal us that we have picked one up as soon
   * as it can. As such, as soon as we get this signal, reset the FSM and switch
   * states.
   */
  if (controller::foraging_signal::BLOCK_PICKUP == data->signal()) {
    if (!m_block_fsm.task_finished()) {
      ER_WARN("WARNING: BLOCK_PICKUP received while still acquiring block--possibly spurious");
    }
    m_block_fsm.task_reset();
    internal_event(ST_TRANSPORT_TO_CACHE);
  } else {
    m_block_fsm.task_execute();
  }
  return controller::foraging_signal::HANDLED;
}
HFSM_STATE_DEFINE(block_to_cache_fsm, transport_to_cache, state_machine::event_data) {
  ER_ASSERT(state_machine::event_type::NORMAL == data->type(), "Bad event type");

  /*
   * We may get a BLOCK_DROP signal even if we have not yet finished vectoring
   * to our chosen cache, because the signal can trigger as soon as we are on
   * the cache, which might not be the center if the cache > 1 block wide. As
   * such, as soon as we get this signal, reset the FSM and switch states.
   */
    if (controller::foraging_signal::BLOCK_DROP == data->signal()) {
      if (!m_cache_fsm.task_finished()) {
        ER_WARN("WARNING: BLOCK_DROP received while still acquiring cache");
      }
      m_cache_fsm.task_reset();
      internal_event(ST_FINISHED);
    } else {
      m_cache_fsm.task_execute();
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

void block_to_cache_fsm::task_start(const rcppsw::task_allocation::taskable_argument* const arg) {
  const tasks::foraging_signal_argument* const a =
      dynamic_cast<const tasks::foraging_signal_argument* const>(arg);
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
