/**
 * @file dpo_fsm.cpp
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
#include "fordyca/fsm/depth0/dpo_fsm.hpp"

#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/fsm/expstrat/foraging_expstrat.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth0);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
dpo_fsm::dpo_fsm(const controller::block_sel_matrix* const sel_matrix,
                 crfootbot::footbot_saa_subsystem* const saa,
                 ds::dpo_store* const store,
                 std::unique_ptr<expstrat::foraging_expstrat> exp_behavior)
    : util_hfsm(saa, ekST_MAX_STATES),
      ER_CLIENT_INIT("fordyca.fsm.depth0.dpo"),
      HFSM_CONSTRUCT_STATE(leaving_nest, &start),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(block_to_nest, hfsm::top_state()),
      m_block_fsm(sel_matrix, saa, store, std::move(exp_behavior)),
      mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
                   HFSM_STATE_MAP_ENTRY_EX(&block_to_nest),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&leaving_nest,
                                               nullptr,
                                               &entry_leaving_nest,
                                               nullptr)} {
  hfsm::change_parent(ekST_LEAVING_NEST, &start);
}

HFSM_STATE_DEFINE(dpo_fsm, start, rpfsm::event_data* data) {
  /* first time running FSM */
  if (rpfsm::event_type::ekNORMAL == data->type()) {
    internal_event(ekST_BLOCK_TO_NEST);
    return controller::foraging_signal::ekHANDLED;
  }
  if (rpfsm::event_type::ekCHILD == data->type()) {
    if (controller::foraging_signal::ekLEFT_NEST == data->signal()) {
      internal_event(ekST_BLOCK_TO_NEST);
      return controller::foraging_signal::ekHANDLED;
    }
  }
  ER_FATAL_SENTINEL("Unhandled signal");
  return controller::foraging_signal::ekHANDLED;
}

HFSM_STATE_DEFINE(dpo_fsm, block_to_nest, rpfsm::event_data* data) {
  if (nullptr != data &&
      controller::foraging_signal::ekFSM_RUN != data->signal() &&
      rpfsm::event_signal::ekIGNORED != data->signal()) {
    m_block_fsm.inject_event(data->signal(), rpfsm::event_type::ekNORMAL);
    return controller::foraging_signal::ekHANDLED;
  }
  if (m_block_fsm.task_finished()) {
    m_block_fsm.task_reset();
    internal_event(ekST_LEAVING_NEST);
  } else {
    m_block_fsm.task_execute();
  }
  return controller::foraging_signal::ekHANDLED;
}

/*******************************************************************************
 * Collision Metrics
 ******************************************************************************/
RCPPSW_WRAP_DEF(dpo_fsm, in_collision_avoidance, m_block_fsm, const)
RCPPSW_WRAP_DEF(dpo_fsm, entered_collision_avoidance, m_block_fsm, const);
RCPPSW_WRAP_DEF(dpo_fsm, exited_collision_avoidance, m_block_fsm, const);
RCPPSW_WRAP_DEF(dpo_fsm, collision_avoidance_duration, m_block_fsm, const);

RCPPSW_WRAP_DEF(dpo_fsm, avoidance_loc, m_block_fsm, const);

/*******************************************************************************
 * Goal Acquisition Metrics
 ******************************************************************************/
RCPPSW_WRAP_DEF(dpo_fsm, is_exploring_for_goal, m_block_fsm, const);
RCPPSW_WRAP_DEF(dpo_fsm, is_vectoring_to_goal, m_block_fsm, const);
RCPPSW_WRAP_DEF(dpo_fsm, acquisition_goal, m_block_fsm, const);
RCPPSW_WRAP_DEF(dpo_fsm, block_transport_goal, m_block_fsm, const);
RCPPSW_WRAP_DEF(dpo_fsm, goal_acquired, m_block_fsm, const);
RCPPSW_WRAP_DEF(dpo_fsm, acquisition_loc, m_block_fsm, const);
RCPPSW_WRAP_DEF(dpo_fsm, current_explore_loc, m_block_fsm, const);
RCPPSW_WRAP_DEF(dpo_fsm, current_vector_loc, m_block_fsm, const);

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void dpo_fsm::init(void) {
  cfsm::util_hfsm::init();
  m_block_fsm.task_reset();
} /* init() */

void dpo_fsm::run(void) {
  inject_event(controller::foraging_signal::ekFSM_RUN,
               rpfsm::event_type::ekNORMAL);
} /* run() */

NS_END(depth0, fsm, fordyca);
