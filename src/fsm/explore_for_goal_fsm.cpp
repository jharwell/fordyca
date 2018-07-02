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
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/datatypes/color.h>
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/params/fsm_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
namespace kinematics = rcppsw::robotics::kinematics;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
explore_for_goal_fsm::explore_for_goal_fsm(
    const std::shared_ptr<rcppsw::er::server>& server,
    controller::saa_subsystem* const saa,
    std::unique_ptr<controller::explore_behavior> behavior,
    std::function<bool(void)> goal_detect)
    : base_explore_fsm(server, saa, ST_MAX_STATES),
      entry_explore(),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(explore, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      mc_state_map{
          HFSM_STATE_MAP_ENTRY_EX(&start),
          HFSM_STATE_MAP_ENTRY_EX_ALL(&explore, nullptr, &entry_explore, nullptr),
          HFSM_STATE_MAP_ENTRY_EX(&finished)},
      m_explore_behavior(std::move(behavior)),
      m_goal_detect(goal_detect) {
  insmod("explore_for_goal_fsm",
         rcppsw::er::er_lvl::DIAG,
         rcppsw::er::er_lvl::NOM);
}

HFSM_STATE_DEFINE_ND(explore_for_goal_fsm, start) {
  internal_event(ST_EXPLORE);
  return controller::foraging_signal::HANDLED;
}

__rcsw_const HFSM_STATE_DEFINE_ND(explore_for_goal_fsm, finished) {
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE_ND(explore_for_goal_fsm, explore) {
  if (ST_EXPLORE != last_state()) {
    ER_DIAG("Executing ST_EXPLORE");
  }

  if (m_goal_detect()) {
    internal_event(ST_FINISHED);
  } else {
    m_explore_behavior->execute();
  }
  return controller::foraging_signal::HANDLED;
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
bool explore_for_goal_fsm::task_running(void) const {
  return ST_START != current_state() && ST_FINISHED != current_state();
}

NS_END(fsm, fordyca);
