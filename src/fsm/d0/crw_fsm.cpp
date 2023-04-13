/**
 * \file crw_fsm.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/d0/crw_fsm.hpp"

#include "cosm/spatial/strategy/base_strategy.hpp"
#include "cosm/subsystem/actuation_subsystem.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystem.hpp"

#include "fordyca/fsm/foraging_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, d0);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
crw_fsm::crw_fsm(const csfsm::fsm_params* params,
                 cffsm::strategy_set strategies,
                 const rmath::vector2d& nest_loc,
                 rmath::rng* rng)
    : foraging_util_hfsm(params,
                         std::move(strategies),
                         rng,
                         ekST_MAX_STATES),
      ER_CLIENT_INIT("fordyca.fsm.d0.crw"),
      RCPPSW_HFSM_CONSTRUCT_STATE(transport_to_nest, &start),
      RCPPSW_HFSM_CONSTRUCT_STATE(drop_carried_block, &start),
      RCPPSW_HFSM_CONSTRUCT_STATE(leaving_nest, &start),
      RCPPSW_HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(acquire_block, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(wait_for_block_pickup, hfsm::top_state()),
      entry_drop_carried_block(),
      RCPPSW_HFSM_CONSTRUCT_STATE(wait_for_block_drop, hfsm::top_state()),
      RCPPSW_HFSM_DEFINE_STATE_MAP(
          mc_state_map,
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&start),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&acquire_block),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&transport_to_nest,
                                             nullptr,
                                             &entry_transport_to_nest,
                                             &exit_transport_to_nest),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&leaving_nest,
                                             nullptr,
                                             &entry_leaving_nest,
                                             &exit_leaving_nest),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_pickup,
                                             nullptr,
                                             &entry_wait_for_signal,
                                             nullptr),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&wait_for_block_drop,
                                             nullptr,
                                             &entry_wait_for_signal,
                                             nullptr),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&drop_carried_block,
                                             nullptr,
                                             &entry_drop_carried_block,
                                             &exit_drop_carried_block)),
      mc_nest_loc(nest_loc),
      m_explore_fsm(params,
                    std::move(foraging_util_hfsm::strategies().explore),
                    rng,
                    std::bind(&crw_fsm::block_detected, this)) {}

/*******************************************************************************
 * States
 ******************************************************************************/
RCPPSW_HFSM_STATE_DEFINE(crw_fsm, start, rpfsm::event_data* data) {
  /* first time running FSM */
  if (rpfsm::event_type::ekNORMAL == data->type()) {
    internal_event(ekST_ACQUIRE_BLOCK);
    return fsm::foraging_signal::ekHANDLED;
  }
  if (rpfsm::event_type::ekCHILD == data->type()) {
    if (fsm::foraging_signal::ekLEFT_NEST == data->signal()) {
      task_reset();
      internal_event(ekST_ACQUIRE_BLOCK);
      return fsm::foraging_signal::ekHANDLED;
    } else if (fsm::foraging_signal::ekENTERED_NEST == data->signal()) {
      internal_event(ekST_WAIT_FOR_BLOCK_DROP);
      return fsm::foraging_signal::ekHANDLED;
    } else if (fsm::foraging_signal::ekDROPPED_BLOCK == data->signal()) {
      /*
       * We can't just switch states--ekST_LEAVING_NEST checks if we are still
       * in a child signal context, so we need to explicitly say that we have
       * handled the signal and things are back to normal.
       */
      data->signal(fsm::foraging_signal::ekRUN);
      data->type(rpfsm::event_type::ekNORMAL);
      internal_event(ekST_LEAVING_NEST);
      return fsm::foraging_signal::ekHANDLED;
    }
  }
  ER_FATAL_SENTINEL("Unhandled signal %d", data->signal());
  return fsm::foraging_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(crw_fsm, acquire_block) {
  if (!m_explore_fsm.task_running()) {
    m_explore_fsm.task_reset();
    m_explore_fsm.task_start(nullptr);
  }
  if (m_explore_fsm.task_running()) {
    m_explore_fsm.task_execute();
    if (m_explore_fsm.task_finished()) {
      internal_event(ekST_WAIT_FOR_BLOCK_PICKUP);
    }
  }
  return fsm::foraging_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE(crw_fsm,
                         wait_for_block_pickup,
                         rpfsm::event_data* data) {
  ER_DEBUG("Waiting for block pickup: signal=%d", data->signal());

  if (fsm::foraging_signal::ekBLOCK_PICKUP == data->signal()) {
    m_explore_fsm.task_reset();
    ER_INFO("Block pickup signal received");
    internal_event(ekST_TRANSPORT_TO_NEST,
                   std::make_unique<nest_transport_data>(mc_nest_loc));
    m_pickup_wait_count = 0;
  } else if (fsm::foraging_signal::ekBLOCK_VANISHED == data->signal()) {
    ER_INFO("Block vanished signal received");
    internal_event(ekST_ACQUIRE_BLOCK);
  }
  ER_ASSERT(++m_pickup_wait_count < 10,
            "Waited too long for block pickup signal");
  return fsm::foraging_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE(crw_fsm,
                         wait_for_block_drop,
                         rpfsm::event_data* data) {
  ER_DEBUG("Wait for block drop signal: signal=%d", data->signal());

  if (ffsm::foraging_signal::ekBLOCK_DROP == data->signal()) {
    m_explore_fsm.task_reset();
    ER_INFO("Block drop signal  received");
    internal_event(ekST_DROP_CARRIED_BLOCK);
  }
  return fsm::foraging_signal::ekHANDLED;
}

RCPPSW_HFSM_ENTRY_DEFINE_ND(crw_fsm, entry_drop_carried_block) {
  ER_DEBUG("Begin dropping block");
}

RCPPSW_HFSM_EXIT_DEFINE(crw_fsm, exit_drop_carried_block) {
  ER_DEBUG("Finished dropping block");
  m_task_finished = true;
}


/*******************************************************************************
 * Goal Acquisition Metrics
 ******************************************************************************/
crw_fsm::exp_status crw_fsm::is_exploring_for_goal(void) const {
  return exp_status{ ekST_ACQUIRE_BLOCK == current_state() ||
                         ekST_LEAVING_NEST == current_state(),
                     true };
} /* is_exploring_for_goal() */

bool crw_fsm::goal_acquired(void) const {
  if (foraging_acq_goal::ekBLOCK == acquisition_goal()) {
    return current_state() == ekST_WAIT_FOR_BLOCK_PICKUP;
  } else if (foraging_transport_goal::ekNEST == block_transport_goal()) {
    return current_state() == ekST_WAIT_FOR_BLOCK_DROP;
  }
  return false;
} /* goal_acquired() */

boost::optional<rmath::vector3z> crw_fsm::acquisition_loc3D(void) const {
  if (goal_acquired()) {
    return boost::make_optional(saa()->sensing()->dpos3D());
  }
  return boost::none;
} /* acquisition_loc3D() */

boost::optional<rmath::vector3z> crw_fsm::explore_loc3D(void) const {
  if (is_exploring_for_goal().is_exploring) {
    return boost::make_optional(saa()->sensing()->dpos3D());
  }
  return boost::none;
} /* explore_loc3D() */

boost::optional<rmath::vector3z> crw_fsm::vector_loc3D(void) const {
  ER_FATAL_SENTINEL("CRW_FSM current vector location undefined");
  return boost::none;
} /* vector_loc3D() */

rtypes::type_uuid crw_fsm::entity_acquired_id(void) const {
  /* CRW FSM has no concept of state, so it doesn't know what it has acquired */
  return rtypes::constants::kNoUUID;
} /* entity_acquired_id() */

csmetrics::goal_acq_metrics::goal_type crw_fsm::acquisition_goal(void) const {
  if (ekST_ACQUIRE_BLOCK == current_state() ||
      ekST_WAIT_FOR_BLOCK_PICKUP == current_state()) {
    return fsm::to_goal_type(foraging_acq_goal::ekBLOCK);
  }
  return fsm::to_goal_type(foraging_acq_goal::ekNONE);
} /* block_transport_goal() */

/*******************************************************************************
 * Block Transport Metrics
 ******************************************************************************/
bool crw_fsm::is_phototaxiing_to_goal(bool include_ca) const {
  if (include_ca) {
    return foraging_transport_goal::ekNEST == block_transport_goal();
  } else {
    return foraging_transport_goal::ekNEST == block_transport_goal() &&
           !exp_interference();
  }
} /* is_phototaxiing_to_goal() */

foraging_transport_goal crw_fsm::block_transport_goal(void) const {
  if (ekST_TRANSPORT_TO_NEST == current_state() ||
      ekST_WAIT_FOR_BLOCK_DROP == current_state() ||
      ekST_DROP_CARRIED_BLOCK == current_state()) {
    return foraging_transport_goal::ekNEST;
  }
  return foraging_transport_goal::ekNONE;
} /* block_transport_goal() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void crw_fsm::init(void) {
  cffsm::foraging_util_hfsm::init();
  m_explore_fsm.init();
} /* init() */

void crw_fsm::run(void) {
  if (event_data_hold() && (nullptr != event_data())) {
    auto* data = event_data();
    data->signal(fsm::foraging_signal::ekRUN);
    data->type(rpfsm::event_type::ekNORMAL);
    inject_event(event_data_release());
  } else {
    inject_event(fsm::foraging_signal::ekRUN, rpfsm::event_type::ekNORMAL);
  }
} /* run() */

bool crw_fsm::block_detected(void) {
  return saa()->sensing()->env()->detect("block");
} /* block_detected() */

/*******************************************************************************
 * Taskable Interface
 ******************************************************************************/
void crw_fsm::task_reset(void) {
  init();
  m_explore_fsm.task_reset();
  m_task_finished = false;
} /* task_reset() */

NS_END(d0, fsm, fordyca);
