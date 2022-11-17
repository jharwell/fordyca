/**
 * \file acquire_new_cache_fsm.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/d2/block_to_new_cache_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, d2);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_to_new_cache_fsm::block_to_new_cache_fsm(
    const fsm_ro_params* c_ro,
    const csfsm::fsm_params* c_no,
    cffsm::strategy_set strategies,
    rmath::rng* rng)
    : block_to_goal_fsm(&m_cache_fsm, &m_block_fsm, c_no, rng),
      m_cache_fsm(c_ro, c_no, strategies.explore->clone(), rng),
      m_block_fsm(c_ro, c_no, strategies.explore->clone(), rng) {}

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
csmetrics::goal_acq_metrics::goal_type
block_to_new_cache_fsm::acquisition_goal(void) const {
  if (ekST_ACQUIRE_BLOCK == current_state() ||
      ekST_WAIT_FOR_BLOCK_PICKUP == current_state()) {
    return fsm::to_goal_type(foraging_acq_goal::ekBLOCK);
  } else if (ekST_TRANSPORT_TO_GOAL == current_state() ||
             ekST_WAIT_FOR_BLOCK_DROP == current_state()) {
    return fsm::to_goal_type(foraging_acq_goal::ekNEW_CACHE);
  }
  return fsm::to_goal_type(foraging_acq_goal::ekNONE);
} /* acquisition_goal() */

foraging_transport_goal block_to_new_cache_fsm::block_transport_goal(void) const {
  if (ekST_TRANSPORT_TO_GOAL == current_state() ||
      ekST_WAIT_FOR_BLOCK_DROP == current_state()) {
    return foraging_transport_goal::ekNEW_CACHE;
  }
  return foraging_transport_goal::ekNONE;
} /* acquisition_goal() */

rtypes::type_uuid block_to_new_cache_fsm::entity_acquired_id(void) const {
  if (foraging_acq_goal::ekBLOCK == acquisition_goal()) {
    return m_block_fsm.entity_acquired_id();
  }
  return rtypes::constants::kNoUUID;
} /* entity_acquired_id() */

NS_END(d2, controller, fordyca);
