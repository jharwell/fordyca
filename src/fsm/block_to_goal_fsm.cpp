/**
 * \file block_to_goal_fsm.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#include "cosm/spatial/fsm/acquire_goal_fsm.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"

#include "fordyca/fsm/foraging_acq_goal.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/tasks/argument.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_to_goal_fsm::block_to_goal_fsm(csfsm::acquire_goal_fsm* const goal_fsm,
                                     csfsm::acquire_goal_fsm* const block_fsm,
                                     csubsystem::saa_subsystemQ3D* saa,
                                     rmath::rng* rng)
    : ER_CLIENT_INIT("fordyca.fsm.block_to_goal"),
      foraging_util_hfsm(saa, nullptr, rng, ekST_MAX_STATES),
      RCPPSW_HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(acquire_block, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(wait_for_block_pickup, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(transport_to_goal, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(wait_for_block_drop, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      RCPPSW_HFSM_DEFINE_STATE_MAP(
          mc_state_map,
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&start),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&acquire_block),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_pickup,
                                             nullptr,
                                             &entry_wait_for_signal,
                                             nullptr),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&transport_to_goal),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_drop,
                                             nullptr,
                                             &entry_wait_for_signal,
                                             nullptr),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&finished)),
      m_goal_fsm(goal_fsm),
      m_block_fsm(block_fsm) {}

RCPPSW_HFSM_STATE_DEFINE(block_to_goal_fsm, start, rpfsm::event_data* data) {
  if (rpfsm::event_type::ekNORMAL == data->type()) {
    if (fsm::foraging_signal::ekACQUIRE_FREE_BLOCK == data->signal() ||
        fsm::foraging_signal::ekACQUIRE_CACHED_BLOCK == data->signal()) {
      internal_event(ekST_ACQUIRE_BLOCK);
      return fsm::foraging_signal::ekHANDLED;
    }
  } else {
    ER_FATAL_SENTINEL("Unhandled child signal %d", data->signal());
    return fsm::foraging_signal::ekUNHANDLED;
  }
  return fsm::foraging_signal::ekHANDLED;
}
RCPPSW_HFSM_STATE_DEFINE_ND(block_to_goal_fsm, acquire_block) {
  if (m_block_fsm->task_finished()) {
    internal_event(ekST_WAIT_FOR_BLOCK_PICKUP);
  } else {
    m_block_fsm->task_execute();
  }
  return fsm::foraging_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(block_to_goal_fsm, transport_to_goal) {
  if (m_goal_fsm->task_finished()) {
    m_goal_fsm->task_reset();
    saa()->actuation()->actuator<ckin2D::governed_diff_drive>()->reset();
    internal_event(ekST_WAIT_FOR_BLOCK_DROP);
  } else {
    m_goal_fsm->task_execute();
  }
  return fsm::foraging_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE(block_to_goal_fsm,
                         wait_for_block_pickup,
                         rpfsm::event_data* data) {
  if (fsm::foraging_signal::ekBLOCK_PICKUP == data->signal()) {
    ER_DEBUG("Block pickup signal received");
    m_block_fsm->task_reset();
    internal_event(ekST_TRANSPORT_TO_GOAL);
    return fsm::foraging_signal::ekHANDLED;
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
  if (fsm::foraging_signal::ekBLOCK_VANISHED == data->signal() ||
      fsm::foraging_signal::ekCACHE_VANISHED == data->signal()) {
    m_block_fsm->task_reset();
    internal_event(ekST_ACQUIRE_BLOCK);
  }
  return fsm::foraging_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE(block_to_goal_fsm,
                         wait_for_block_drop,
                         rpfsm::event_data* data) {
  saa()->actuation()->actuator<ckin2D::governed_diff_drive>()->reset();
  if (fsm::foraging_signal::ekBLOCK_DROP == data->signal()) {
    ER_DEBUG("Block drop signal received");
    internal_event(ekST_FINISHED);
  } else if (fsm::foraging_signal::ekBLOCK_PROXIMITY == data->signal()) {
    ER_ASSERT(foraging_acq_goal::ekCACHE_SITE == acquisition_goal(),
              "Bad goal on block proximity");
    m_goal_fsm->task_reset();
    internal_event(ekST_TRANSPORT_TO_GOAL);
  } else if (fsm::foraging_signal::ekCACHE_VANISHED == data->signal()) {
    ER_ASSERT(foraging_acq_goal::ekEXISTING_CACHE == acquisition_goal(),
              "Non-existing cache vanished?");
    m_goal_fsm->task_reset();
    internal_event(ekST_TRANSPORT_TO_GOAL);
  } else if (fsm::foraging_signal::ekCACHE_PROXIMITY == data->signal()) {
    ER_ASSERT(foraging_acq_goal::ekNEW_CACHE == acquisition_goal() ||
                  foraging_acq_goal::ekCACHE_SITE == acquisition_goal(),
              "Bad goal on cache proxmity");
    m_goal_fsm->task_reset();
    internal_event(ekST_TRANSPORT_TO_GOAL);
  }
  return fsm::foraging_signal::ekHANDLED;
}

RCPPSW_CONST RCPPSW_HFSM_STATE_DEFINE_ND(block_to_goal_fsm, finished) {
  return fsm::foraging_signal::ekHANDLED;
}

/*******************************************************************************
 * Interference Metrics
 ******************************************************************************/
bool block_to_goal_fsm::exp_interference(void) const {
  return (m_block_fsm->task_running() && m_block_fsm->exp_interference()) ||
         (m_goal_fsm->task_running() && m_goal_fsm->exp_interference());
} /* exp_interference() */

bool block_to_goal_fsm::entered_interference(void) const {
  return (m_block_fsm->task_running() && m_block_fsm->entered_interference()) ||
         (m_goal_fsm->task_running() && m_goal_fsm->entered_interference());
} /* entered_interference() */

bool block_to_goal_fsm::exited_interference(void) const {
  return (m_block_fsm->task_running() && m_block_fsm->exited_interference()) ||
         (m_goal_fsm->task_running() && m_goal_fsm->exited_interference());
} /* exited_interference() */

rtypes::timestep block_to_goal_fsm::interference_duration(void) const {
  if (m_block_fsm->task_running()) {
    return m_block_fsm->interference_duration();
  } else if (m_goal_fsm->task_running()) {
    return m_goal_fsm->interference_duration();
  }
  return rtypes::timestep(0);
} /* interference_duration() */

rmath::vector3z block_to_goal_fsm::interference_loc3D(void) const {
  ER_ASSERT(m_block_fsm->task_running() || m_goal_fsm->task_running(),
            "In collision interference without running task?");
  if (m_block_fsm->task_running()) {
    return m_block_fsm->interference_loc3D();
  } else { /* goal FSM must be running */
    return m_goal_fsm->interference_loc3D();
  }
} /* interference_loc3D() */

/*******************************************************************************
 * Acquisition Metrics
 ******************************************************************************/
block_to_goal_fsm::exp_status
block_to_goal_fsm::is_exploring_for_goal(void) const {
  if (m_block_fsm->task_running()) {
    return m_block_fsm->is_exploring_for_goal();
  } else if (m_goal_fsm->task_running()) {
    return m_goal_fsm->is_exploring_for_goal();
  }
  return exp_status{ false, false };
} /* is_exploring_for_goal() */

bool block_to_goal_fsm::is_vectoring_to_goal(void) const {
  return (m_block_fsm->is_vectoring_to_goal() && m_block_fsm->task_running()) ||
         (m_goal_fsm->is_vectoring_to_goal() && m_goal_fsm->task_running());
} /* is_vectoring_to_block */

bool block_to_goal_fsm::goal_acquired(void) const {
  return (ekST_WAIT_FOR_BLOCK_PICKUP == current_state()) ||
         (ekST_WAIT_FOR_BLOCK_DROP == current_state());
} /* goal_acquired() */

csmetrics::goal_acq_metrics::goal_type
block_to_goal_fsm::acquisition_goal(void) const {
  if (m_block_fsm->task_running()) {
    return m_block_fsm->acquisition_goal();
  } else if (m_goal_fsm->task_running()) {
    return m_goal_fsm->acquisition_goal();
  }
  return fsm::to_goal_type(foraging_acq_goal::ekNONE);
} /* acquisition_goal() */

rmath::vector3z block_to_goal_fsm::acquisition_loc3D(void) const {
  return m_goal_fsm->acquisition_loc3D();
} /* acquisition_loc3D() */

rmath::vector3z block_to_goal_fsm::explore_loc3D(void) const {
  return saa()->sensing()->dpos3D();
} /* explore_loc3D() */

rmath::vector3z block_to_goal_fsm::vector_loc3D(void) const {
  return saa()->sensing()->dpos3D();
} /* vector_loc3D() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void block_to_goal_fsm::init(void) {
  util_hfsm::init();
  m_goal_fsm->task_reset();
  m_block_fsm->task_reset();
} /* init() */

void block_to_goal_fsm::task_start(cta::taskable_argument* const arg) {
  const auto* a = dynamic_cast<const tasks::foraging_signal_argument*>(arg);
  ER_ASSERT(nullptr != a, "Bad argument passed");
  inject_event(a->signal(), rpfsm::event_type::ekNORMAL);
}

void block_to_goal_fsm::task_execute(void) {
  inject_event(fsm::foraging_signal::ekRUN, rpfsm::event_type::ekNORMAL);
} /* task_execute() */

NS_END(fsm, fordyca);
