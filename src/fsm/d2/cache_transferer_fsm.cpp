/**
 * \file cache_transferer_fsm.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/d2/cache_transferer_fsm.hpp"

#include "fordyca/fsm/foraging_acq_goal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, d2);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
cache_transferer_fsm::cache_transferer_fsm(
    const fsm_ro_params* c_ro,
    const csfsm::fsm_params* c_no,
    cffsm::strategy_set strategies,
    rmath::rng* rng)
    : block_to_goal_fsm(&m_dest_cache_fsm, &m_src_cache_fsm, c_no, rng),
      m_src_cache_fsm(c_ro, c_no, strategies.explore->clone(), rng, true),
      m_dest_cache_fsm(c_ro, c_no, strategies.explore->clone(), rng, false) {}

/*******************************************************************************
 * Block Acquisition Metrics
 ******************************************************************************/
csmetrics::goal_acq_metrics::goal_type
cache_transferer_fsm::acquisition_goal(void) const {
  if (ekST_START != current_state() && ekST_FINISHED != current_state()) {
    return fsm::to_goal_type(foraging_acq_goal::ekEXISTING_CACHE);
  }
  return fsm::to_goal_type(foraging_acq_goal::ekNONE);
} /* acquisition_goal() */

foraging_transport_goal cache_transferer_fsm::block_transport_goal(void) const {
  if (ekST_TRANSPORT_TO_GOAL == current_state() ||
      ekST_WAIT_FOR_BLOCK_DROP == current_state()) {
    return foraging_transport_goal::ekEXISTING_CACHE;
  }
  return foraging_transport_goal::ekNONE;
} /* block_transport_goal() */

bool cache_transferer_fsm::is_acquiring_dest_cache(void) const {
  return foraging_transport_goal::ekEXISTING_CACHE == block_transport_goal() &&
         m_dest_cache_fsm.task_running();
} /* is_acquiring_dest_cache() */

bool cache_transferer_fsm::is_acquiring_src_cache(void) const {
  return foraging_transport_goal::ekEXISTING_CACHE == block_transport_goal() &&
         m_src_cache_fsm.task_running();
} /* is_acquiring_src_cache() */

rtypes::type_uuid cache_transferer_fsm::entity_acquired_id(void) const {
  if (is_acquiring_dest_cache()) {
    return m_dest_cache_fsm.entity_acquired_id();
  } else if (is_acquiring_src_cache()) {
    return m_src_cache_fsm.entity_acquired_id();
  }
  return rtypes::constants::kNoUUID;
} /* entity_acquired_id() */

NS_END(d2, fsm, fordyca);
