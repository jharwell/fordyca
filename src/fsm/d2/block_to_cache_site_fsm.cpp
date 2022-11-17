/**
 * \file block_to_cache_site_fsm.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/d2/block_to_cache_site_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, d2);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_to_cache_site_fsm::block_to_cache_site_fsm(
    const fsm_ro_params* c_ro,
    const csfsm::fsm_params* c_no,
    cffsm::strategy_set strategies,
    rmath::rng* rng)
    : block_to_goal_fsm(&m_cache_fsm, &m_block_fsm, c_no, rng),
      m_cache_fsm(c_ro, c_no, rng),
      m_block_fsm(c_ro, c_no, std::move(strategies.explore), rng) {}

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
csmetrics::goal_acq_metrics::goal_type
block_to_cache_site_fsm::acquisition_goal(void) const {
  if (ekST_ACQUIRE_BLOCK == current_state() ||
      ekST_WAIT_FOR_BLOCK_PICKUP == current_state()) {
    return fsm::to_goal_type(foraging_acq_goal::ekBLOCK);
  } else if (ekST_TRANSPORT_TO_GOAL == current_state() ||
             ekST_WAIT_FOR_BLOCK_DROP == current_state()) {
    return fsm::to_goal_type(foraging_acq_goal::ekCACHE_SITE);
  }
  return fsm::to_goal_type(foraging_acq_goal::ekNONE);
} /* acquisition_goal() */

foraging_transport_goal
block_to_cache_site_fsm::block_transport_goal(void) const {
  if (ekST_TRANSPORT_TO_GOAL == current_state() ||
      ekST_WAIT_FOR_BLOCK_DROP == current_state()) {
    return foraging_transport_goal::ekCACHE_SITE;
  }
  return foraging_transport_goal::ekNONE;
} /* acquisition_goal() */

NS_END(d2, controller, fordyca);
