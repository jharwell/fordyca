/**
 * \file free_block_to_nest_fsm.cpp
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
#include "fordyca/fsm/d0/free_block_to_nest_fsm.hpp"

#include "cosm/subsystem/saa_subsystemQ3D.hpp"

#include "fordyca/controller/cognitive/block_sel_matrix.hpp"
#include "fordyca/fsm/foraging_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, d0);

using goal_type = csmetrics::goal_acq_metrics::goal_type;
using bsel_matrix = controller::cognitive::block_sel_matrix;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
free_block_to_nest_fsm::free_block_to_nest_fsm(
    const fsm_ro_params* c_ro,
    const csfsm::fsm_params* c_no,
    cffsm::strategy_set strategies,
    rmath::rng* rng)
    : foraging_util_hfsm(c_no, std::move(strategies), rng, ekST_MAX_STATES),
      ER_CLIENT_INIT("fordyca.fsm.d0.free_block_to_nest"),
      RCPPSW_HFSM_CONSTRUCT_STATE(leaving_nest, &start),
      RCPPSW_HFSM_CONSTRUCT_STATE(transport_to_nest, &start),
      RCPPSW_HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(acquire_block, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(wait_for_pickup, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(wait_for_drop, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      RCPPSW_HFSM_DEFINE_STATE_MAP(
          mc_state_map,
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&start),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&acquire_block),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_pickup,
                                             nullptr,
                                             &entry_wait_for_signal,
                                             nullptr),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_drop,
                                             nullptr,
                                             &entry_wait_for_signal,
                                             nullptr),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&transport_to_nest,
                                             nullptr,
                                             &entry_transport_to_nest,
                                             &exit_transport_to_nest),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&finished)),
      mc_nest_loc(std::get<rmath::vector2d>(
          c_ro->bsel_matrix->find(bsel_matrix::kNestLoc)->second)),
      m_block_fsm(c_ro,
                  c_no,
                  std::move(foraging_util_hfsm::strategies().explore),
                  rng) {}

RCPPSW_HFSM_STATE_DEFINE(free_block_to_nest_fsm, start, rpfsm::event_data* data) {
  /* first time running FSM */
  if (rpfsm::event_type::ekNORMAL == data->type()) {
    internal_event(ekST_ACQUIRE_BLOCK);
    return fsm::foraging_signal::ekHANDLED;
  }
  if (rpfsm::event_type::ekCHILD == data->type()) {
    if (fsm::foraging_signal::ekENTERED_NEST == data->signal()) {
      internal_event(ekST_WAIT_FOR_DROP);
      return fsm::foraging_signal::ekHANDLED;
    }
  }
  ER_FATAL_SENTINEL("Unhandled signal %d", data->signal());
  return fsm::foraging_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(free_block_to_nest_fsm, acquire_block) {
  if (m_block_fsm.task_finished()) {
    internal_event(ekST_WAIT_FOR_PICKUP);
  } else {
    m_block_fsm.task_execute();
  }
  return fsm::foraging_signal::ekHANDLED;
}
RCPPSW_HFSM_STATE_DEFINE(free_block_to_nest_fsm,
                         wait_for_pickup,
                         rpfsm::event_data* data) {
  /**
   * It is possible that robots can be waiting indefinitely for a block
   * pickup signal that will never come once a block has been acquired if they
   * "detect" a block by sprawling across multiple blocks (i.e. all ground
   * sensors did not detect the same block). It is also possible that a robot
   * serving a penalty for a block pickup will have the block taken by a
   * different robot.
   *
   * In both cases, treat the block as vanished and try again.
   */
  if (fsm::foraging_signal::ekBLOCK_PICKUP == data->signal()) {
    m_block_fsm.task_reset();
    internal_event(ekST_TRANSPORT_TO_NEST,
                   std::make_unique<nest_transport_data>(mc_nest_loc));
  } else if (fsm::foraging_signal::ekBLOCK_VANISHED == data->signal()) {
    m_block_fsm.task_reset();
    internal_event(ekST_ACQUIRE_BLOCK);
  }
  return fsm::foraging_signal::ekHANDLED;
}
RCPPSW_HFSM_STATE_DEFINE(free_block_to_nest_fsm,
                         wait_for_drop,
                         rpfsm::event_data* data) {
  if (fsm::foraging_signal::ekBLOCK_DROP == data->signal()) {
    m_block_fsm.task_reset();
    internal_event(ekST_FINISHED);
  }
  return fsm::foraging_signal::ekHANDLED;
}

RCPPSW_CONST RCPPSW_HFSM_STATE_DEFINE_ND(free_block_to_nest_fsm, finished) {
  return fsm::foraging_signal::ekHANDLED;
}

/*******************************************************************************
 * Goal Acquisition Metrics
 ******************************************************************************/
RCPPSW_WRAP_DEF_OVERRIDE(free_block_to_nest_fsm,
                         is_exploring_for_goal,
                         m_block_fsm,
                         const);
RCPPSW_WRAP_DEF_OVERRIDE(free_block_to_nest_fsm,
                         is_vectoring_to_goal,
                         m_block_fsm,
                         const);

RCPPSW_WRAP_DEF_OVERRIDE(free_block_to_nest_fsm,
                         acquisition_loc3D,
                         m_block_fsm,
                         const);

RCPPSW_WRAP_DEF_OVERRIDE(free_block_to_nest_fsm,
                         explore_loc3D,
                         m_block_fsm,
                         const);
RCPPSW_WRAP_DEF_OVERRIDE(free_block_to_nest_fsm, vector_loc3D, m_block_fsm, const);

RCPPSW_WRAP_DEF_OVERRIDE(free_block_to_nest_fsm,
                         entity_acquired_id,
                         m_block_fsm,
                         const);
goal_type free_block_to_nest_fsm::acquisition_goal(void) const {
  if (ekST_ACQUIRE_BLOCK == current_state() ||
      ekST_WAIT_FOR_PICKUP == current_state()) {
    return fsm::to_goal_type(foraging_acq_goal::ekBLOCK);
  }
  return fsm::to_goal_type(foraging_acq_goal::ekNONE);
} /* acquisition_goal() */

bool free_block_to_nest_fsm::goal_acquired(void) const {
  if (foraging_acq_goal::ekBLOCK == acquisition_goal()) {
    return current_state() == ekST_WAIT_FOR_PICKUP;
  } else if (foraging_transport_goal::ekNEST == block_transport_goal()) {
    return current_state() == ekST_WAIT_FOR_DROP;
  }
  return false;
} /* goal_acquired() */

/*******************************************************************************
 * Block Transport Metrics
 ******************************************************************************/
bool free_block_to_nest_fsm::is_phototaxiing_to_goal(bool include_ca) const {
  if (include_ca) {
    return foraging_transport_goal::ekNEST == block_transport_goal();
  } else {
    return foraging_transport_goal::ekNEST == block_transport_goal() &&
           !exp_interference();
  }
} /* is_phototaxiing_to_goal() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void free_block_to_nest_fsm::init(void) {
  foraging_util_hfsm::init();
  m_block_fsm.task_reset();
} /* init() */

void free_block_to_nest_fsm::task_execute(void) {
  if (event_data_hold() && (nullptr != event_data())) {
    auto* data = event_data();
    data->signal(fsm::foraging_signal::ekRUN);
    data->type(rpfsm::event_type::ekNORMAL);
    inject_event(event_data_release());
  } else {
    inject_event(fsm::foraging_signal::ekRUN, rpfsm::event_type::ekNORMAL);
  }
} /* task_execute() */

foraging_transport_goal free_block_to_nest_fsm::block_transport_goal(void) const {
  if (ekST_TRANSPORT_TO_NEST == current_state() ||
      ekST_WAIT_FOR_DROP == current_state()) {
    return foraging_transport_goal::ekNEST;
  }
  return foraging_transport_goal::ekNONE;
} /* acquisition_goal() */

NS_END(d0, fsm, fordyca);
