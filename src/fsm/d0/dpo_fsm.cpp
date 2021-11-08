/**
 * \file dpo_fsm.cpp
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
#include "fordyca/fsm/d0/dpo_fsm.hpp"

#include "cosm/subsystem/saa_subsystemQ3D.hpp"

#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/strategy/foraging_strategy.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, d0);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
dpo_fsm::dpo_fsm(const fsm_ro_params* c_ro,
                 const csfsm::fsm_params* c_no,
                 std::unique_ptr<csstrategy::base_strategy> explore,
                 std::unique_ptr<cssnest_acq::base_nest_acq> nest_acq,
                 rmath::rng* rng)
    : foraging_util_hfsm(c_no, nullptr, rng, ekST_MAX_STATES),
      ER_CLIENT_INIT("fordyca.fsm.d0.dpo"),
      RCPPSW_HFSM_CONSTRUCT_STATE(leaving_nest, &start),
      RCPPSW_HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(block_to_nest, hfsm::top_state()),
      RCPPSW_HFSM_DEFINE_STATE_MAP(
          mc_state_map,
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&start),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&block_to_nest),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&leaving_nest,
                                             nullptr,
                                             &entry_leaving_nest,
                                             nullptr)),
      m_block_fsm(c_ro, c_no, std::move(explore), std::move(nest_acq), rng) {
  hfsm::change_parent(ekST_LEAVING_NEST, &start);
}

RCPPSW_HFSM_STATE_DEFINE(dpo_fsm, start, rpfsm::event_data* data) {
  /* first time running FSM */
  if (rpfsm::event_type::ekNORMAL == data->type()) {
    internal_event(ekST_BLOCK_TO_NEST);
    return fsm::foraging_signal::ekHANDLED;
  }
  if (rpfsm::event_type::ekCHILD == data->type()) {
    if (fsm::foraging_signal::ekLEFT_NEST == data->signal()) {
      internal_event(ekST_BLOCK_TO_NEST);
      return fsm::foraging_signal::ekHANDLED;
    }
  }
  ER_FATAL_SENTINEL("Unhandled signal");
  return fsm::foraging_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE(dpo_fsm, block_to_nest, rpfsm::event_data* data) {
  if (nullptr != data && fsm::foraging_signal::ekRUN != data->signal() &&
      rpfsm::event_signal::ekIGNORED != data->signal()) {
    m_block_fsm.inject_event(data->signal(), rpfsm::event_type::ekNORMAL);
    return fsm::foraging_signal::ekHANDLED;
  }
  if (m_block_fsm.task_finished()) {
    m_task_finished = true;
    m_block_fsm.task_reset();
    internal_event(ekST_LEAVING_NEST);
  } else {
    m_block_fsm.task_execute();
  }
  return fsm::foraging_signal::ekHANDLED;
}

/*******************************************************************************
 * Goal Acquisition Metrics
 ******************************************************************************/
RCPPSW_WRAP_DEF(dpo_fsm, is_exploring_for_goal, m_block_fsm, const);
RCPPSW_WRAP_DEF(dpo_fsm, is_vectoring_to_goal, m_block_fsm, const);
RCPPSW_WRAP_DEF(dpo_fsm, acquisition_goal, m_block_fsm, const);
RCPPSW_WRAP_DEF(dpo_fsm, block_transport_goal, m_block_fsm, const);
RCPPSW_WRAP_DEF(dpo_fsm, goal_acquired, m_block_fsm, const);
RCPPSW_WRAP_DEF(dpo_fsm, acquisition_loc3D, m_block_fsm, const);
RCPPSW_WRAP_DEF(dpo_fsm, explore_loc3D, m_block_fsm, const);
RCPPSW_WRAP_DEF(dpo_fsm, vector_loc3D, m_block_fsm, const);
RCPPSW_WRAP_DEF(dpo_fsm, entity_acquired_id, m_block_fsm, const);

bool dpo_fsm::is_phototaxiing_to_goal(bool include_ca) const {
  return m_block_fsm.is_phototaxiing_to_goal(include_ca);
} /* is_phototaxiing_to_goal() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void dpo_fsm::init(void) {
  csfsm::util_hfsm::init();
  m_block_fsm.task_reset();
} /* init() */

void dpo_fsm::run(void) {
  m_task_finished = false;
  inject_event(fsm::foraging_signal::ekRUN, rpfsm::event_type::ekNORMAL);
} /* run() */

NS_END(d0, fsm, fordyca);
