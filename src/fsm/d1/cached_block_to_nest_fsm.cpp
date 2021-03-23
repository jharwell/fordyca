/**
 * \file cached_block_to_nest_fsm.cpp
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
#include "fordyca/fsm/d1/cached_block_to_nest_fsm.hpp"

#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"

#include "fordyca/controller/cognitive/cache_sel_matrix.hpp"
#include "fordyca/strategy/foraging_strategy.hpp"
#include "fordyca/fsm/foraging_acq_goal.hpp"
#include "fordyca/fsm/foraging_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, d1);

using csel_matrix = controller::cognitive::cache_sel_matrix;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
cached_block_to_nest_fsm::cached_block_to_nest_fsm(
    const fsm_ro_params* const c_params,
    crfootbot::footbot_saa_subsystem* saa,
    std::unique_ptr<csstrategy::base_strategy> explore,
    std::unique_ptr<csstrategy::base_strategy> nest_acq,
    rmath::rng* rng)
    : foraging_util_hfsm(saa, std::move(nest_acq), rng, ekST_MAX_STATES),
      ER_CLIENT_INIT("fordyca.fsm.d1.cached_block_to_nest"),
      RCPPSW_HFSM_CONSTRUCT_STATE(transport_to_nest, &start),
      RCPPSW_HFSM_CONSTRUCT_STATE(leaving_nest, &start),
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
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&leaving_nest,
                                             nullptr,
                                             &entry_leaving_nest,
                                             nullptr),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&finished)),
      mc_nest_loc(boost::get<rmath::vector2d>(
          c_params->csel_matrix->find(csel_matrix::kNestLoc)->second)),
      m_cache_fsm(c_params, saa, std::move(explore), rng, true) {}

RCPPSW_HFSM_STATE_DEFINE(cached_block_to_nest_fsm,
                         start,
                         rpfsm::event_data* data) {
  if (rpfsm::event_type::ekNORMAL == data->type()) {
    internal_event(ekST_ACQUIRE_BLOCK);
    return fsm::foraging_signal::ekHANDLED;
  } else if (rpfsm::event_type::ekCHILD == data->type()) {
    if (fsm::foraging_signal::ekENTERED_NEST == data->signal()) {
      internal_event(ekST_WAIT_FOR_DROP);
      return fsm::foraging_signal::ekHANDLED;
    } else if (fsm::foraging_signal::ekLEFT_NEST == data->signal()) {
      internal_event(ekST_ACQUIRE_BLOCK);
      return fsm::foraging_signal::ekHANDLED;
    }
  }
  ER_FATAL_SENTINEL("Unhandled signal %d", data->signal());
  return fsm::foraging_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(cached_block_to_nest_fsm, acquire_block) {
  if (m_cache_fsm.task_finished()) {
    internal_event(ekST_WAIT_FOR_PICKUP);
  } else {
    m_cache_fsm.task_execute();
  }
  return fsm::foraging_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE(cached_block_to_nest_fsm,
                         wait_for_pickup,
                         rpfsm::event_data* data) {
  if (fsm::foraging_signal::ekBLOCK_PICKUP == data->signal()) {
    m_cache_fsm.task_reset();
    internal_event(ekST_TRANSPORT_TO_NEST,
                   std::make_unique<nest_transport_data>(mc_nest_loc));
  } else if (fsm::foraging_signal::ekCACHE_VANISHED == data->signal()) {
    m_cache_fsm.task_reset();
    internal_event(ekST_ACQUIRE_BLOCK);
  }
  return fsm::foraging_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE(cached_block_to_nest_fsm,
                         wait_for_drop,
                         rpfsm::event_data* data) {
  if (fsm::foraging_signal::ekBLOCK_DROP == data->signal()) {
    m_cache_fsm.task_reset();
    internal_event(ekST_FINISHED);
  }
  return fsm::foraging_signal::ekHANDLED;
}

RCPPSW_CONST RCPPSW_HFSM_STATE_DEFINE_ND(cached_block_to_nest_fsm, finished) {
  return fsm::foraging_signal::ekHANDLED;
}

/*******************************************************************************
 * Collision Metrics
 ******************************************************************************/
bool cached_block_to_nest_fsm::exp_interference(void) const {
  return (m_cache_fsm.task_running() && m_cache_fsm.exp_interference()) ||
         cffsm::foraging_util_hfsm::exp_interference();
} /* in_interference() */

bool cached_block_to_nest_fsm::entered_interference(void) const {
  return (m_cache_fsm.task_running() && m_cache_fsm.entered_interference()) ||
         cffsm::foraging_util_hfsm::entered_interference();
} /* entered_interference() */

bool cached_block_to_nest_fsm::exited_interference(void) const {
  return (m_cache_fsm.task_running() && m_cache_fsm.exited_interference()) ||
         cffsm::foraging_util_hfsm::exited_interference();
} /* exited_interference() */

rtypes::timestep cached_block_to_nest_fsm::interference_duration(void) const {
  if (m_cache_fsm.task_running()) {
    return m_cache_fsm.interference_duration();
  } else {
    return cffsm::foraging_util_hfsm::interference_duration();
  }
} /* interference_duration() */

rmath::vector3z cached_block_to_nest_fsm::interference_loc3D(void) const {
  if (m_cache_fsm.task_running()) {
    return m_cache_fsm.interference_loc3D();
  } else {
    return cffsm::foraging_util_hfsm::interference_loc3D();
  }
} /* interference_loc3D() */

/*******************************************************************************
 * Block Acquisition Metrics
 ******************************************************************************/
RCPPSW_WRAP_DEF_OVERRIDE(cached_block_to_nest_fsm,
                         is_exploring_for_goal,
                         m_cache_fsm,
                         const);

RCPPSW_WRAP_DEF_OVERRIDE(cached_block_to_nest_fsm,
                         is_vectoring_to_goal,
                         m_cache_fsm,
                         const);

RCPPSW_WRAP_DEF_OVERRIDE(cached_block_to_nest_fsm,
                         acquisition_loc3D,
                         m_cache_fsm,
                         const);

RCPPSW_WRAP_DEF_OVERRIDE(cached_block_to_nest_fsm,
                         explore_loc3D,
                         m_cache_fsm,
                         const);
RCPPSW_WRAP_DEF_OVERRIDE(cached_block_to_nest_fsm,
                         vector_loc3D,
                         m_cache_fsm,
                         const);

RCPPSW_WRAP_DEF_OVERRIDE(cached_block_to_nest_fsm,
                         entity_acquired_id,
                         m_cache_fsm,
                         const);

bool cached_block_to_nest_fsm::goal_acquired(void) const {
  if (foraging_acq_goal::ekEXISTING_CACHE == acquisition_goal()) {
    return current_state() == ekST_WAIT_FOR_PICKUP;
  } else if (foraging_transport_goal::ekNEST == block_transport_goal()) {
    return current_state() == ekST_WAIT_FOR_DROP;
  }
  return false;
}

csmetrics::goal_acq_metrics::goal_type
cached_block_to_nest_fsm::acquisition_goal(void) const {
  if (ekST_ACQUIRE_BLOCK == current_state() ||
      ekST_WAIT_FOR_PICKUP == current_state()) {
    return fsm::to_goal_type(foraging_acq_goal::ekEXISTING_CACHE);
  } else if (ekST_ACQUIRE_BLOCK == current_state() ||
             ekST_WAIT_FOR_PICKUP == current_state()) {
    return fsm::to_goal_type(foraging_acq_goal::ekEXISTING_CACHE);
  }
  return fsm::to_goal_type(foraging_acq_goal::ekNONE);
} /* acquisition_goal() */

/*******************************************************************************
 * Block Transport Metrics
 ******************************************************************************/
bool cached_block_to_nest_fsm::is_phototaxiing_to_goal(bool include_ca) const {
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
foraging_transport_goal
cached_block_to_nest_fsm::block_transport_goal(void) const {
  if (ekST_TRANSPORT_TO_NEST == current_state() ||
      ekST_WAIT_FOR_DROP == current_state()) {
    return foraging_transport_goal::ekNEST;
  }
  return foraging_transport_goal::ekNONE;
} /* block_transport_goal() */

void cached_block_to_nest_fsm::init(void) {
  foraging_util_hfsm::init();
  m_cache_fsm.task_reset();
} /* init() */

void cached_block_to_nest_fsm::task_execute(void) {
  if (event_data_hold() && (nullptr != event_data())) {
    auto* data = event_data();
    data->signal(fsm::foraging_signal::ekRUN);
    data->type(rpfsm::event_type::ekNORMAL);
    inject_event(event_data_release());
  } else {
    inject_event(fsm::foraging_signal::ekRUN, rpfsm::event_type::ekNORMAL);
  }
} /* task_execute() */

NS_END(d1, fsm, fordyca);
