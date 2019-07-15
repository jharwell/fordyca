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
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/fsm/acquire_goal_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_to_goal_fsm::block_to_goal_fsm(acquire_goal_fsm* const goal_fsm,
                                     acquire_goal_fsm* const block_fsm,
                                     controller::saa_subsystem* saa)
    : ER_CLIENT_INIT("fordyca.fsm.block_to_goal"),
      base_foraging_fsm(saa, ekST_MAX_STATES),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_block, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_block_pickup, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(transport_to_goal, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(wait_for_block_drop, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      m_goal_fsm(goal_fsm),
      m_block_fsm(block_fsm),
      mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
                   HFSM_STATE_MAP_ENTRY_EX(&acquire_block),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_pickup,
                                               nullptr,
                                               &entry_wait_for_signal,
                                               nullptr),
                   HFSM_STATE_MAP_ENTRY_EX(&transport_to_goal),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_drop,
                                               nullptr,
                                               &entry_wait_for_signal,
                                               nullptr),
                   HFSM_STATE_MAP_ENTRY_EX(&finished)} {}

HFSM_STATE_DEFINE(block_to_goal_fsm, start, rpfsm::event_data* data) {
  if (rpfsm::event_type::ekNORMAL == data->type()) {
    if (controller::foraging_signal::ekACQUIRE_FREE_BLOCK == data->signal() ||
        controller::foraging_signal::ekACQUIRE_CACHED_BLOCK == data->signal()) {
      internal_event(ekST_ACQUIRE_BLOCK);
      return controller::foraging_signal::ekHANDLED;
    }
  } else {
    ER_FATAL_SENTINEL("Unhandled child signal %d", data->signal());
    return controller::foraging_signal::ekUNHANDLED;
  }
  return controller::foraging_signal::ekHANDLED;
}
HFSM_STATE_DEFINE_ND(block_to_goal_fsm, acquire_block) {
  if (m_block_fsm->task_finished()) {
    internal_event(ekST_WAIT_FOR_BLOCK_PICKUP);
  } else {
    m_block_fsm->task_execute();
  }
  return controller::foraging_signal::ekHANDLED;
}

HFSM_STATE_DEFINE_ND(block_to_goal_fsm, transport_to_goal) {
  if (m_goal_fsm->task_finished()) {
    m_goal_fsm->task_reset();
    saa_subsystem()->actuation()->differential_drive().stop();
    internal_event(ekST_WAIT_FOR_BLOCK_DROP);
  } else {
    m_goal_fsm->task_execute();
  }
  return controller::foraging_signal::ekHANDLED;
}

HFSM_STATE_DEFINE(block_to_goal_fsm,
                  wait_for_block_pickup,
                  rpfsm::event_data* data) {
  if (controller::foraging_signal::ekBLOCK_PICKUP == data->signal()) {
    ER_DEBUG("Block pickup signal received");
    m_block_fsm->task_reset();
    internal_event(ekST_TRANSPORT_TO_GOAL);
    return controller::foraging_signal::ekHANDLED;
  }
  /**
   * It is possible that robots can be waiting indefinitely for a block
   * pickup signal that will never come once a block has been acquired if they
   * "detect" a block by sprawling across multiple blocks (i.e. all ground
   * sensors did not detect the same block). It is also possible that a robot
   * serving a penalty for a block pickup will have the block taken by a
   * different robot.
   *
   * Similar things can happen for a cache to vanish while a robot is waiting to
   * pick up a block from it.
   */
  if (controller::foraging_signal::ekBLOCK_VANISHED == data->signal() ||
      controller::foraging_signal::ekCACHE_VANISHED == data->signal()) {
    m_block_fsm->task_reset();
    internal_event(ekST_ACQUIRE_BLOCK);
  }
  return controller::foraging_signal::ekHANDLED;
}

HFSM_STATE_DEFINE(block_to_goal_fsm,
                  wait_for_block_drop,
                  rpfsm::event_data* data) {
  saa_subsystem()->actuation()->differential_drive().stop();
  if (controller::foraging_signal::ekBLOCK_DROP == data->signal()) {
    ER_DEBUG("Block drop signal received");
    internal_event(ekST_FINISHED);
  } else if (controller::foraging_signal::ekBLOCK_PROXIMITY == data->signal()) {
    ER_ASSERT(acq_goal_type::ekCACHE_SITE == acquisition_goal(),
              "Bad goal on block proximity");
    m_goal_fsm->task_reset();
    internal_event(ekST_TRANSPORT_TO_GOAL);
  } else if (controller::foraging_signal::ekCACHE_VANISHED == data->signal()) {
    ER_ASSERT(acq_goal_type::ekEXISTING_CACHE == acquisition_goal(),
              "Non-existing cache vanished? ");
    m_goal_fsm->task_reset();
    internal_event(ekST_TRANSPORT_TO_GOAL);
  } else if (controller::foraging_signal::ekCACHE_PROXIMITY == data->signal()) {
    ER_ASSERT(acq_goal_type::ekNEW_CACHE == acquisition_goal() ||
                  acq_goal_type::ekCACHE_SITE == acquisition_goal(),
              "Bad goal on cache proxmity");
    m_goal_fsm->task_reset();
    internal_event(ekST_TRANSPORT_TO_GOAL);
  }
  return controller::foraging_signal::ekHANDLED;
}

RCSW_CONST HFSM_STATE_DEFINE_ND(block_to_goal_fsm, finished) {
  return controller::foraging_signal::ekHANDLED;
}

/*******************************************************************************
 * Collision Metrics
 ******************************************************************************/
 bool block_to_goal_fsm::in_collision_avoidance(void) const {
  return (m_block_fsm->task_running() && m_block_fsm->in_collision_avoidance()) ||
         (m_goal_fsm->task_running() && m_goal_fsm->in_collision_avoidance());
} /* in_collision_avoidance() */

 bool block_to_goal_fsm::entered_collision_avoidance(void) const {
  return (m_block_fsm->task_running() &&
          m_block_fsm->entered_collision_avoidance()) ||
         (m_goal_fsm->task_running() &&
          m_goal_fsm->entered_collision_avoidance());
} /* entered_collision_avoidance() */

 bool block_to_goal_fsm::exited_collision_avoidance(void) const {
  return (m_block_fsm->task_running() &&
          m_block_fsm->exited_collision_avoidance()) ||
         (m_goal_fsm->task_running() &&
          m_goal_fsm->exited_collision_avoidance());
} /* exited_collision_avoidance() */

 rtypes::timestep block_to_goal_fsm::collision_avoidance_duration(
    void) const {
  if (m_block_fsm->task_running()) {
    return m_block_fsm->collision_avoidance_duration();
  } else if (m_goal_fsm->task_running()) {
    return m_goal_fsm->collision_avoidance_duration();
  }
  return rtypes::timestep(0);
} /* collision_avoidance_duration() */

rmath::vector2u block_to_goal_fsm::avoidance_loc(void) const {
  ER_ASSERT(m_block_fsm->task_running() || m_goal_fsm->task_running(),
            "In collision avoidance without running task?");
  if (m_block_fsm->task_running()) {
    return m_block_fsm->avoidance_loc();
  } else { /* goal FSM must be running */
    return m_goal_fsm->avoidance_loc();
  }
} /* avoidance_loc() */

/*******************************************************************************
 * Acquisition Metrics
 ******************************************************************************/
 block_to_goal_fsm::exp_status block_to_goal_fsm::is_exploring_for_goal(
    void) const {
  if (m_block_fsm->task_running()) {
    return m_block_fsm->is_exploring_for_goal();
  } else if (m_goal_fsm->task_running()) {
    return m_goal_fsm->is_exploring_for_goal();
  }
  return std::make_pair(false, false);
} /* is_exploring_for_goal() */

 bool block_to_goal_fsm::is_vectoring_to_goal(void) const {
  return (m_block_fsm->is_vectoring_to_goal() && m_block_fsm->task_running()) ||
         (m_goal_fsm->is_vectoring_to_goal() && m_goal_fsm->task_running());
} /* is_vectoring_to_block */

 bool block_to_goal_fsm::goal_acquired(void) const {
  return (ekST_WAIT_FOR_BLOCK_PICKUP == current_state()) ||
         (ekST_WAIT_FOR_BLOCK_DROP == current_state());
} /* goal_acquired() */

acq_goal_type block_to_goal_fsm::acquisition_goal(void) const {
  if (m_block_fsm->task_running()) {
    return m_block_fsm->acquisition_goal();
  } else if (m_goal_fsm->task_running()) {
    return m_goal_fsm->acquisition_goal();
  }
  return acq_goal_type::ekNONE;
} /* acquisition_goal() */

rmath::vector2u block_to_goal_fsm::acquisition_loc(void) const {
  return m_goal_fsm->acquisition_loc();
} /* acquisition_loc() */

rmath::vector2u block_to_goal_fsm::current_explore_loc(void) const {
  return saa_subsystem()->sensing()->discrete_position();
} /* current_explore_loc() */

rmath::vector2u block_to_goal_fsm::current_vector_loc(void) const {
  return saa_subsystem()->sensing()->discrete_position();
} /* current_vector_loc() */
/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void block_to_goal_fsm::init(void) {
  base_foraging_fsm::init();
  m_goal_fsm->task_reset();
  m_block_fsm->task_reset();
} /* init() */

void block_to_goal_fsm::task_start(const rta::taskable_argument* const arg) {
  auto* a = dynamic_cast<const tasks::foraging_signal_argument*>(arg);
  ER_ASSERT(nullptr != a, "Bad argument passed");
  inject_event(a->signal(), rpfsm::event_type::ekNORMAL);
}

void block_to_goal_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::ekFSM_RUN,
               rpfsm::event_type::ekNORMAL);
} /* task_execute() */

NS_END(fsm, fordyca);
