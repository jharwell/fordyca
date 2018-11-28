/**
 * @file acquire_goal_fsm.cpp
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
#include "fordyca/fsm/acquire_goal_fsm.hpp"

#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/controller/random_explore_behavior.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/representation/base_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_goal_fsm::acquire_goal_fsm(
    controller::saa_subsystem* saa,
    const acquisition_goal_ftype& acquisition_goal,
    const goal_candidates_ftype& candidates_exist_cb,
    const goal_select_ftype& goal_select,
    const std::function<bool(bool)>& goal_acquired_cb,
    const std::function<bool(void)>& explore_term_cb)
    : base_foraging_fsm(saa, ST_MAX_STATES),
      ER_CLIENT_INIT("forydca.fsm.acquire_goal_fsm"),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(fsm_acquire_goal, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      exit_fsm_acquire_goal(),
      m_vector_fsm(saa),
      m_explore_fsm(saa,
                    std::make_unique<controller::random_explore_behavior>(saa),
                    explore_term_cb),
      m_acquisition_goal(acquisition_goal),
      m_candidates_exist(candidates_exist_cb),
      m_goal_select(goal_select),
      m_goal_acquired_cb(goal_acquired_cb),
      mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&fsm_acquire_goal,
                                               nullptr,
                                               nullptr,
                                               &exit_fsm_acquire_goal),
                   HFSM_STATE_MAP_ENTRY_EX(&finished)} {
  m_explore_fsm.change_parent(explore_for_goal_fsm::ST_EXPLORE,
                              &fsm_acquire_goal);
}

HFSM_STATE_DEFINE_ND(acquire_goal_fsm, start) {
  ER_DEBUG("Executing ST_START");
  internal_event(ST_ACQUIRE_GOAL);
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE_ND(acquire_goal_fsm, fsm_acquire_goal) {
  if (ST_ACQUIRE_GOAL != last_state()) {
    ER_DEBUG("Executing ST_ACQUIRE_GOAL");
  }

  if (acquire_goal()) {
    internal_event(ST_FINISHED);
  }
  return state_machine::event_signal::HANDLED;
}

HFSM_EXIT_DEFINE(acquire_goal_fsm, exit_fsm_acquire_goal) {
  m_vector_fsm.task_reset();
  m_explore_fsm.task_reset();
}
HFSM_STATE_DEFINE_ND(acquire_goal_fsm, finished) {
  if (ST_FINISHED != last_state()) {
    ER_DEBUG("Executing ST_FINISHED");
  }

  return state_machine::event_signal::HANDLED;
}

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
__rcsw_pure bool acquire_goal_fsm::in_collision_avoidance(void) const {
  return m_explore_fsm.in_collision_avoidance() ||
         m_vector_fsm.in_collision_avoidance();
} /* in_collision_avoidance() */

__rcsw_pure bool acquire_goal_fsm::entered_collision_avoidance(void) const {
  return m_explore_fsm.entered_collision_avoidance() ||
         m_vector_fsm.entered_collision_avoidance();
} /* entered_collision_avoidance() */

__rcsw_pure bool acquire_goal_fsm::exited_collision_avoidance(void) const {
  return m_explore_fsm.exited_collision_avoidance() ||
         m_vector_fsm.exited_collision_avoidance();
} /* exited_collision_avoidance() */

__rcsw_pure uint acquire_goal_fsm::collision_avoidance_duration(void) const {
  return m_explore_fsm.collision_avoidance_duration() ||
         m_vector_fsm.collision_avoidance_duration();
} /* collision_avoidance_duration() */

bool acquire_goal_fsm::goal_acquired(void) const {
  return current_state() == ST_FINISHED;
} /* cache_acquired() */

bool acquire_goal_fsm::is_exploring_for_goal(void) const {
  return (current_state() == ST_ACQUIRE_GOAL && m_explore_fsm.task_running());
} /* is_exploring_for_goal() */

bool acquire_goal_fsm::is_vectoring_to_goal(void) const {
  return current_state() == ST_ACQUIRE_GOAL && m_vector_fsm.task_running();
} /* is_vectoring_to_goal() */

acquisition_goal_type acquire_goal_fsm::acquisition_goal(void) const {
  if (ST_ACQUIRE_GOAL == current_state()) {
    return m_acquisition_goal();
  }
  return acquisition_goal_type::kNone;
} /* acquisition_goal() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void acquire_goal_fsm::init(void) {
  base_foraging_fsm::init();
  m_vector_fsm.task_reset();
  m_explore_fsm.task_reset();
} /* init() */

bool acquire_goal_fsm::acquire_goal(void) {
  /*
   * If we know of goal caches in the arena, go to the location of the best
   * one. Otherwise, explore until you find one. If during exploration we find
   * one through our LOS, then stop exploring and go vector to it.
   */
  if (!acquire_known_goal()) {
    if (m_vector_fsm.task_running()) {
      return false;
    }

    /*
     * When we got to our chosen goal it was found to be unsuitable and we
     * currently don't know of any other candidates, so we have to explore to
     * find what we want.
     */
    return acquire_unknown_goal();
  }
  return true;
} /* acquire_goal() */

bool acquire_goal_fsm::acquire_unknown_goal(void) {
  if (!m_explore_fsm.task_running()) {
    m_explore_fsm.task_reset();
    m_explore_fsm.task_start(nullptr);
  }
  m_explore_fsm.task_execute();
  if (m_explore_fsm.task_finished()) {
    return m_goal_acquired_cb(true);
  }
  return false;
} /* acquire_unknown_goal() */

bool acquire_goal_fsm::acquire_known_goal(void) {
  /*
   * If we don't know of any blocks and we are not current vectoring towards
   * one, then there is no way we can acquire a known block, so bail out.
   */
  if (!m_candidates_exist() && !m_vector_fsm.task_running()) {
    return false;
  }

  if (m_candidates_exist() && !m_vector_fsm.task_running()) {
    /*
     * If we get here, we must know of some candidates/perceived entities of
     * interest, but not be currently vectoring toward any of them.
     */
    auto selection = m_goal_select();

    /*
     * If this happens, all the entities we know of are too close for us to
     * vector to, or there was some other issue with selecting one.
     */
    if (false == std::get<0>(selection)) {
      return false;
    }

    m_explore_fsm.task_reset();
    m_vector_fsm.task_reset();

    ER_INFO("Start acquiring goal@%s tol=%f",
            std::get<1>(selection).to_str().c_str(),
            std::get<2>(selection));
    tasks::vector_argument v(std::get<2>(selection), std::get<1>(selection));
    m_vector_fsm.task_start(&v);
  }

  /* we are vectoring */
  if (!m_vector_fsm.task_finished()) {
    m_vector_fsm.task_execute();
  }

  if (m_vector_fsm.task_finished()) {
    m_vector_fsm.task_reset();
    return m_goal_acquired_cb(false);
  }
  return false;
} /* acquire_known_block() */

void acquire_goal_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* task_execute() */

NS_END(fsm, fordyca);
