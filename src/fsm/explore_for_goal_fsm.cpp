/**
 * @file explore_for_goal_fsm.cpp
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
#include "fordyca/fsm/explore_for_goal_fsm.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/controller/saa_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
namespace kinematics = rcppsw::robotics::kinematics;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
explore_for_goal_fsm::explore_for_goal_fsm(
    controller::saa_subsystem* const saa,
    std::unique_ptr<controller::explore_behavior> behavior,
    std::function<bool(void)> goal_detect)
    : base_explore_fsm(saa, ST_MAX_STATES),
      ER_CLIENT_INIT("fordyca.fsm.explore_for_goal"),
      entry_explore(),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(explore, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      mc_state_map{
          HFSM_STATE_MAP_ENTRY_EX(&start),
          HFSM_STATE_MAP_ENTRY_EX_ALL(&explore, nullptr, &entry_explore, nullptr),
          HFSM_STATE_MAP_ENTRY_EX(&finished)},
      m_explore_behavior(std::move(behavior)),
      m_goal_detect(goal_detect) {}

HFSM_STATE_DEFINE_ND(explore_for_goal_fsm, start) {
  internal_event(ST_EXPLORE);
  return controller::foraging_signal::HANDLED;
}

__rcsw_const HFSM_STATE_DEFINE_ND(explore_for_goal_fsm, finished) {
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE_ND(explore_for_goal_fsm, explore) {
  if (ST_EXPLORE != last_state()) {
    ER_DEBUG("Executing ST_EXPLORE");
    m_explore_time = 0;
  }

  if (m_explore_time >= kMIN_EXPLORE_TIME && m_goal_detect()) {
    internal_event(ST_FINISHED);
  } else {
    m_explore_behavior->execute();
    ++m_explore_time;
  }
  return controller::foraging_signal::HANDLED;
}

/*******************************************************************************
 * Collision Metrics
 ******************************************************************************/
FSM_OVERRIDE_DEF(bool,
                 explore_for_goal_fsm,
                 in_collision_avoidance,
                 *m_explore_behavior, const);
FSM_OVERRIDE_DEF(bool,
                 explore_for_goal_fsm,
                 entered_collision_avoidance,
                 *m_explore_behavior, const);
FSM_OVERRIDE_DEF(bool,
                 explore_for_goal_fsm,
                 exited_collision_avoidance,
                 *m_explore_behavior, const);
FSM_OVERRIDE_DEF(uint,
                 explore_for_goal_fsm,
                 collision_avoidance_duration,
                 *m_explore_behavior, const);

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
bool explore_for_goal_fsm::task_running(void) const {
  return ST_START != current_state() && ST_FINISHED != current_state();
}

NS_END(fsm, fordyca);
