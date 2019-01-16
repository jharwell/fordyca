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
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/foraging_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth0);
namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
dpo_fsm::dpo_fsm(const controller::block_sel_matrix* const sel_matrix,
                 controller::saa_subsystem* const saa,
                 ds::dpo_store* const store)
    : base_foraging_fsm(saa, ST_MAX_STATES),
      ER_CLIENT_INIT("fordyca.fsm.depth0.dpo"),
      HFSM_CONSTRUCT_STATE(leaving_nest, &start),
      entry_leaving_nest(),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(block_to_nest, hfsm::top_state()),
      m_block_fsm(sel_matrix, saa, store),
      mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
                   HFSM_STATE_MAP_ENTRY_EX(&block_to_nest),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&leaving_nest,
                                               nullptr,
                                               &entry_leaving_nest,
                                               nullptr)} {
  hfsm::change_parent(ST_LEAVING_NEST, &start);
}

HFSM_STATE_DEFINE(dpo_fsm, start, state_machine::event_data) {
  /* first time running FSM */
  if (state_machine::event_type::NORMAL == data->type()) {
    internal_event(ST_BLOCK_TO_NEST);
    return controller::foraging_signal::HANDLED;
  }
  if (state_machine::event_type::CHILD == data->type()) {
    if (controller::foraging_signal::LEFT_NEST == data->signal()) {
      internal_event(ST_BLOCK_TO_NEST);
      return controller::foraging_signal::HANDLED;
    }
  }
  ER_FATAL_SENTINEL("Unhandled signal");
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE(dpo_fsm, block_to_nest, state_machine::event_data) {
  if (nullptr != data &&
      controller::foraging_signal::FSM_RUN != data->signal() &&
      state_machine::event_signal::IGNORED != data->signal()) {
    m_block_fsm.inject_event(data->signal(), state_machine::event_type::NORMAL);
    return controller::foraging_signal::HANDLED;
  }
  if (m_block_fsm.task_finished()) {
    m_block_fsm.task_reset();
    internal_event(ST_LEAVING_NEST);
  } else {
    m_block_fsm.task_execute();
  }
  return controller::foraging_signal::HANDLED;
}

/*******************************************************************************
 * Collision Metrics
 ******************************************************************************/
__rcsw_pure FSM_OVERRIDE_DEF(bool,
                             dpo_fsm,
                             in_collision_avoidance,
                             m_block_fsm,
                             const);
__rcsw_pure FSM_OVERRIDE_DEF(bool,
                             dpo_fsm,
                             entered_collision_avoidance,
                             m_block_fsm,
                             const);
__rcsw_pure FSM_OVERRIDE_DEF(bool,
                             dpo_fsm,
                             exited_collision_avoidance,
                             m_block_fsm,
                             const);
__rcsw_pure FSM_OVERRIDE_DEF(uint,
                             dpo_fsm,
                             collision_avoidance_duration,
                             m_block_fsm,
                             const);

/*******************************************************************************
 * Goal Acquisition Metrics
 ******************************************************************************/
FSM_OVERRIDE_DEF(bool, dpo_fsm, is_exploring_for_goal, m_block_fsm, const);
FSM_OVERRIDE_DEF(bool, dpo_fsm, is_vectoring_to_goal, m_block_fsm, const);
FSM_OVERRIDE_DEF(acquisition_goal_type,
                 dpo_fsm,
                 acquisition_goal,
                 m_block_fsm,
                 const);
FSM_OVERRIDE_DEF(transport_goal_type,
                 dpo_fsm,
                 block_transport_goal,
                 m_block_fsm,
                 const);
FSM_OVERRIDE_DEF(bool, dpo_fsm, goal_acquired, m_block_fsm, const);

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void dpo_fsm::init(void) {
  base_foraging_fsm::init();
  m_block_fsm.task_reset();
} /* init() */

void dpo_fsm::run(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* run() */

NS_END(depth0, fsm, fordyca);