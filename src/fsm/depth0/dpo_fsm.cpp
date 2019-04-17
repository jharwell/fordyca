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
namespace rfsm = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
dpo_fsm::dpo_fsm(const controller::block_sel_matrix* const sel_matrix,
                 controller::saa_subsystem* const saa,
                 ds::dpo_store* const store)
    : base_foraging_fsm(saa, kST_MAX_STATES),
      ER_CLIENT_INIT("fordyca.fsm.depth0.dpo"),
      HFSM_CONSTRUCT_STATE(leaving_nest, &start),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(block_to_nest, hfsm::top_state()),
      m_block_fsm(sel_matrix, saa, store),
      mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
                   HFSM_STATE_MAP_ENTRY_EX(&block_to_nest),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&leaving_nest,
                                               nullptr,
                                               &entry_leaving_nest,
                                               nullptr)} {
  hfsm::change_parent(kST_LEAVING_NEST, &start);
}

HFSM_STATE_DEFINE(dpo_fsm, start, rfsm::event_data* data) {
  /* first time running FSM */
  if (rfsm::event_type::kNORMAL == data->type()) {
    internal_event(kST_BLOCK_TO_NEST);
    return controller::foraging_signal::kHANDLED;
  }
  if (rfsm::event_type::kCHILD == data->type()) {
    if (controller::foraging_signal::kLEFT_NEST == data->signal()) {
      internal_event(kST_BLOCK_TO_NEST);
      return controller::foraging_signal::kHANDLED;
    }
  }
  ER_FATAL_SENTINEL("Unhandled signal");
  return controller::foraging_signal::kHANDLED;
}

HFSM_STATE_DEFINE(dpo_fsm, block_to_nest, rfsm::event_data* data) {
  if (nullptr != data &&
      controller::foraging_signal::kFSM_RUN != data->signal() &&
      rfsm::event_signal::kIGNORED != data->signal()) {
    m_block_fsm.inject_event(data->signal(), rfsm::event_type::kNORMAL);
    return controller::foraging_signal::kHANDLED;
  }
  if (m_block_fsm.task_finished()) {
    m_block_fsm.task_reset();
    internal_event(kST_LEAVING_NEST);
  } else {
    m_block_fsm.task_execute();
  }
  return controller::foraging_signal::kHANDLED;
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
FSM_OVERRIDE_DEF(rmath::vector2u, dpo_fsm, acquisition_loc, m_block_fsm, const);

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void dpo_fsm::init(void) {
  base_foraging_fsm::init();
  m_block_fsm.task_reset();
} /* init() */

void dpo_fsm::run(void) {
  inject_event(controller::foraging_signal::kFSM_RUN, rfsm::event_type::kNORMAL);
} /* run() */

NS_END(depth0, fsm, fordyca);
