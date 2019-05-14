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
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/controller/random_explore_behavior.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/repr/base_block.hpp"
#include "fordyca/repr/base_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_goal_fsm::acquire_goal_fsm(controller::saa_subsystem* const saa,
                                   const struct hook_list& hooks)
    : base_foraging_fsm(saa, kST_MAX_STATES),
      ER_CLIENT_INIT("forydca.fsm.acquire_goal_fsm"),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(fsm_acquire_goal, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      m_hooks(hooks),
      m_vector_fsm(saa),
      m_explore_fsm(saa,
                    std::make_unique<controller::random_explore_behavior>(saa),
                    m_hooks.explore_term_cb),
      mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&fsm_acquire_goal,
                                               nullptr,
                                               nullptr,
                                               &exit_fsm_acquire_goal),
                   HFSM_STATE_MAP_ENTRY_EX(&finished)} {
  m_explore_fsm.change_parent(explore_for_goal_fsm::kST_EXPLORE,
                              &fsm_acquire_goal);
}

HFSM_STATE_DEFINE_ND(acquire_goal_fsm, start) {
  ER_DEBUG("Executing kST_START");
  internal_event(kST_ACQUIRE_GOAL);
  return controller::foraging_signal::kHANDLED;
}

HFSM_STATE_DEFINE_ND(acquire_goal_fsm, fsm_acquire_goal) {
  if (kST_ACQUIRE_GOAL != last_state()) {
    ER_DEBUG("Executing kST_ACQUIRE_GOAL");
  }

  if (acquire_goal()) {
    internal_event(kST_FINISHED);
  }
  return rfsm::event_signal::kHANDLED;
}

HFSM_EXIT_DEFINE(acquire_goal_fsm, exit_fsm_acquire_goal) {
  m_vector_fsm.task_reset();
  m_explore_fsm.task_reset();
}
HFSM_STATE_DEFINE_ND(acquire_goal_fsm, finished) {
  if (kST_FINISHED != last_state()) {
    ER_DEBUG("Executing kST_FINISHED");
  }

  return rfsm::event_signal::kHANDLED;
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
  if (kST_ACQUIRE_GOAL == current_state() && m_explore_fsm.task_running()) {
    return m_explore_fsm.collision_avoidance_duration();
  } else if (kST_ACQUIRE_GOAL == current_state() && m_vector_fsm.task_running()) {
    return m_vector_fsm.collision_avoidance_duration();
  }
  return 0;
} /* collision_avoidance_duration() */

__rcsw_pure bool acquire_goal_fsm::goal_acquired(void) const {
  return current_state() == kST_FINISHED;
} /* cache_acquired() */

__rcsw_pure bool acquire_goal_fsm::is_exploring_for_goal(void) const {
  return (current_state() == kST_ACQUIRE_GOAL && m_explore_fsm.task_running());
} /* is_exploring_for_goal() */

__rcsw_pure bool acquire_goal_fsm::is_vectoring_to_goal(void) const {
  return current_state() == kST_ACQUIRE_GOAL && m_vector_fsm.task_running();
} /* is_vectoring_to_goal() */

acquisition_goal_type acquire_goal_fsm::acquisition_goal(void) const {
  if (kST_ACQUIRE_GOAL == current_state()) {
    return m_hooks.acquisition_goal();
  }
  return acquisition_goal_type::ekNONE;
} /* acquisition_goal() */

rmath::vector2u acquire_goal_fsm::acquisition_loc(void) const {
  return saa_subsystem()->sensing()->discrete_position();
} /* acquisition_loc() */

rmath::vector2u acquire_goal_fsm::current_explore_loc(void) const {
  return saa_subsystem()->sensing()->discrete_position();
} /* current_explore_loc() */

rmath::vector2u acquire_goal_fsm::current_vector_loc(void) const {
  return saa_subsystem()->sensing()->discrete_position();
} /* current_vector_loc() */

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
    return m_hooks.goal_acquired_cb(true);
  }
  return false;
} /* acquire_unknown_goal() */

bool acquire_goal_fsm::acquire_known_goal(void) {
  /*
   * If we don't know of any blocks and we are not current vectoring towards
   * one, then there is no way we can acquire a known block, so bail out.
   */
  if (!m_hooks.candidates_exist() && !m_vector_fsm.task_running()) {
    return false;
  }

  if (m_hooks.candidates_exist() && !m_vector_fsm.task_running()) {
    /*
     * If we get here, we must know of some candidates/perceived entities of
     * interest, but not be currently vectoring toward any of them.
     */
    auto selection = m_hooks.goal_select();

    /*
     * If this happens, all the entities we know of are too close for us to
     * vector to, or there was some other issue with selecting one.
     */
    if (!selection) {
      return false;
    }

    m_explore_fsm.task_reset();
    m_vector_fsm.task_reset();
    m_acq_id = -1;

    ER_INFO("Start acquiring goal@%s tol=%f",
            std::get<0>(selection.get()).to_str().c_str(),
            std::get<1>(selection.get()));
    tasks::vector_argument v(std::get<1>(selection.get()),
                             std::get<0>(selection.get()));
    m_acq_id = std::get<2>(selection.get());
    m_vector_fsm.task_start(&v);
  }

  /* we are vectoring */
  if (!m_vector_fsm.task_finished()) {
    m_vector_fsm.task_execute();
    if (!m_hooks.goal_valid_cb(m_vector_fsm.target(), m_acq_id)) {
      m_vector_fsm.task_reset();
      return false;
    }
  }

  if (m_vector_fsm.task_finished()) {
    m_vector_fsm.task_reset();
    return m_hooks.goal_acquired_cb(false);
  }
  return false;
} /* acquire_known_block() */

void acquire_goal_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::kFSM_RUN, rfsm::event_type::kNORMAL);
} /* task_execute() */

NS_END(fsm, fordyca);
