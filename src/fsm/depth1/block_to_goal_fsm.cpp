/**
 * @file block_to_goal_fsm.cpp
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
#include "fordyca/fsm/depth1/block_to_goal_fsm.hpp"
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/depth1/sensing_subsystem.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/fsm/acquire_goal_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth1);
namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_to_goal_fsm::block_to_goal_fsm(
    std::shared_ptr<rcppsw::er::server>& server,
    const controller::block_selection_matrix* sel_matrix,
    controller::saa_subsystem* const saa,
    representation::perceived_arena_map* const map)
    : base_foraging_fsm(server, saa, ST_MAX_STATES),
      entry_wait_for_signal(),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_free_block, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_block_pickup, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(transport_to_goal, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_block_drop, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      m_pickup_count(0),
      m_block_fsm(server, sel_matrix, saa, map),
      mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
                   HFSM_STATE_MAP_ENTRY_EX(&acquire_free_block),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_pickup,
                                               nullptr,
                                               &entry_wait_for_signal,
                                               nullptr),
                   HFSM_STATE_MAP_ENTRY_EX(&transport_to_goal),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_drop,
                                               nullptr,
                                               &entry_wait_for_signal,
                                               nullptr),
                   HFSM_STATE_MAP_ENTRY_EX(&finished)} {
  insmod("block_to_goal_fsm", rcppsw::er::er_lvl::DIAG, rcppsw::er::er_lvl::NOM);
}

HFSM_STATE_DEFINE(block_to_goal_fsm, start, state_machine::event_data) {
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
HFSM_STATE_DEFINE_ND(block_to_goal_fsm, acquire_free_block) {
  if (m_block_fsm.task_finished()) {
    internal_event(ST_WAIT_FOR_BLOCK_PICKUP);
  } else {
    m_block_fsm.task_execute();
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE_ND(block_to_goal_fsm, transport_to_goal) {
  if (goal_fsm().task_finished()) {
    goal_fsm().task_reset();
    actuators()->differential_drive().stop();
    internal_event(ST_WAIT_FOR_BLOCK_DROP);
  } else {
    goal_fsm().task_execute();
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE(block_to_goal_fsm,
                  wait_for_block_pickup,
                  state_machine::event_data) {
  if (controller::foraging_signal::BLOCK_PICKUP == data->signal()) {
    ER_DIAG("Block pickup signal received");
    m_block_fsm.task_reset();
    m_pickup_count = 0;
    internal_event(ST_TRANSPORT_TO_GOAL);
    return controller::foraging_signal::HANDLED;
  }
  /**
   * It is possible that robots can be waiting indefinitely for a block
   * pickup signal that will never come once a block has been acquired if they
   * "detect" a block by sprawling across multiple blocks (i.e. all ground
   * sensors did not detect the same block). It is also possible that a robot
   * serving a penalty for a block pickup will have the block taken by a
   * different robot.
   *
   * In both cases, treat the block as vanished and try again.
   */
  if (controller::foraging_signal::BLOCK_PICKUP == data->signal()) {
    m_block_fsm.task_reset();
    internal_event(ST_TRANSPORT_TO_GOAL);
  } else if (controller::foraging_signal::BLOCK_VANISHED == data->signal()) {
    m_block_fsm.task_reset();
    internal_event(ST_ACQUIRE_FREE_BLOCK);
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE(block_to_goal_fsm,
                  wait_for_block_drop,
                  state_machine::event_data) {
  if (controller::foraging_signal::BLOCK_DROP == data->signal()) {
    ER_DIAG("Block drop signal received");
    internal_event(ST_FINISHED);
  } else if (controller::foraging_signal::CACHE_VANISHED == data->signal()) {
    ER_ASSERT(acquisition_goal_type::kExistingCache == acquisition_goal(),
              "FATAL: Non-existing cache vanished? ");
    goal_fsm().task_reset();
    internal_event(ST_TRANSPORT_TO_GOAL);
  } else if (controller::foraging_signal::CACHE_APPEARED == data->signal()) {
    ER_ASSERT(acquisition_goal_type::kNewCache == acquisition_goal() ||
                  acquisition_goal_type::kCacheSite == acquisition_goal(),
              "FATAL: Bad goal on cache appear");
    goal_fsm().task_reset();
    internal_event(ST_TRANSPORT_TO_GOAL);
  }
  return controller::foraging_signal::HANDLED;
}

__rcsw_const HFSM_STATE_DEFINE_ND(block_to_goal_fsm, finished) {
  return controller::foraging_signal::HANDLED;
}

/*******************************************************************************
 * Collision Metrics
 ******************************************************************************/
__rcsw_pure bool block_to_goal_fsm::in_collision_avoidance(void) const {
  return (m_block_fsm.task_running() && m_block_fsm.in_collision_avoidance()) ||
      (goal_fsm().task_running() && goal_fsm().in_collision_avoidance());
} /* in_collision_avoidance() */

__rcsw_pure bool block_to_goal_fsm::entered_collision_avoidance(void) const {
  return (m_block_fsm.task_running() && m_block_fsm.entered_collision_avoidance()) ||
      (goal_fsm().task_running() && goal_fsm().entered_collision_avoidance());
} /* entered_collision_avoidance() */

__rcsw_pure bool block_to_goal_fsm::exited_collision_avoidance(void) const {
  return (m_block_fsm.task_running() && m_block_fsm.exited_collision_avoidance()) ||
      (goal_fsm().task_running() && goal_fsm().exited_collision_avoidance());
} /* exited_collision_avoidance() */

__rcsw_pure uint block_to_goal_fsm::collision_avoidance_duration(void) const {
  if (m_block_fsm.task_running()) {
    return m_block_fsm.collision_avoidance_duration();
  } else if (goal_fsm().task_running()) {
    return goal_fsm().collision_avoidance_duration();
  }
  return 0;
} /* collision_avoidance_duration() */

/*******************************************************************************
 * Acquisition Metrics
 ******************************************************************************/
__rcsw_pure bool block_to_goal_fsm::is_exploring_for_goal(void) const {
  return (m_block_fsm.is_exploring_for_goal() && m_block_fsm.task_running()) ||
         (goal_fsm().is_exploring_for_goal() && goal_fsm().task_running());
} /* is_exploring_for_goal() */

__rcsw_pure bool block_to_goal_fsm::is_vectoring_to_goal(void) const {
  return (m_block_fsm.is_vectoring_to_goal() && m_block_fsm.task_running()) ||
         (goal_fsm().is_vectoring_to_goal() && goal_fsm().task_running());
} /* is_vectoring_to_block */

bool block_to_goal_fsm::goal_acquired(void) const {
  return (ST_WAIT_FOR_BLOCK_PICKUP == current_state()) ||
         (ST_WAIT_FOR_BLOCK_DROP == current_state());
} /* goal_acquired() */

acquisition_goal_type block_to_goal_fsm::acquisition_goal(void) const {
  if (m_block_fsm.task_running()) {
    return m_block_fsm.acquisition_goal();
  } else if (goal_fsm().task_running()) {
    return goal_fsm().acquisition_goal();
  }
  return acquisition_goal_type::kNone;
} /* acquisition_goal() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void block_to_goal_fsm::init(void) {
  base_foraging_fsm::init();
  goal_fsm().task_reset();
  m_block_fsm.task_reset();
} /* init() */

void block_to_goal_fsm::task_start(
    const rcppsw::task_allocation::taskable_argument* const arg) {
  auto* a = dynamic_cast<const tasks::foraging_signal_argument* const>(arg);
  ER_ASSERT(a, "FATAL: bad argument passed");
  inject_event(a->signal(), state_machine::event_type::NORMAL);
}

void block_to_goal_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* task_execute() */

NS_END(depth1, fsm, fordyca);
