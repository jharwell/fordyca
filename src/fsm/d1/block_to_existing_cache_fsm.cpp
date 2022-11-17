/**
 * \file acquire_existing_cache_fsm.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/d1/block_to_existing_cache_fsm.hpp"

#include "cosm/arena/repr/light_type_index.hpp"

#include "fordyca/strategy/explore/block_factory.hpp"
#include "fordyca/strategy/explore/cache_factory.hpp"
#include "fordyca/strategy/foraging_strategy.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, d1);

/*******************************************************************************
 * Forward Decls
 ******************************************************************************/
static acquire_existing_cache_fsm cache_fsm_build(const fsm_ro_params* c_ro,
                                                  const csfsm::fsm_params* c_no,
                                                  rmath::rng* rng);

static acquire_free_block_fsm block_fsm_build(const fsm_ro_params* c_ro,
                                              const csfsm::fsm_params* c_no,
                                              rmath::rng* rng);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_to_existing_cache_fsm::block_to_existing_cache_fsm(
    const fsm_ro_params* c_ro,
    const csfsm::fsm_params* c_no,
    rmath::rng* rng)
    : block_to_goal_fsm(&m_cache_fsm, &m_block_fsm, c_no, rng),
      m_cache_fsm(cache_fsm_build(c_ro, c_no, rng)),
      m_block_fsm(block_fsm_build(c_ro, c_no, rng)) {}

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
csmetrics::goal_acq_metrics::goal_type
block_to_existing_cache_fsm::acquisition_goal(void) const {
  if (ekST_ACQUIRE_BLOCK == current_state() ||
      ekST_WAIT_FOR_BLOCK_PICKUP == current_state()) {
    return fsm::to_goal_type(foraging_acq_goal::ekBLOCK);
  } else if (ekST_TRANSPORT_TO_GOAL == current_state() ||
             ekST_WAIT_FOR_BLOCK_DROP == current_state()) {
    return fsm::to_goal_type(foraging_acq_goal::ekEXISTING_CACHE);
  }
  return fsm::to_goal_type(foraging_acq_goal::ekNONE);
} /* acquisition_goal() */

foraging_transport_goal
block_to_existing_cache_fsm::block_transport_goal(void) const {
  if (ekST_TRANSPORT_TO_GOAL == current_state() ||
      ekST_WAIT_FOR_BLOCK_DROP == current_state()) {
    return foraging_transport_goal::ekEXISTING_CACHE;
  }
  return foraging_transport_goal::ekNONE;
} /* acquisition_goal() */

bool block_to_existing_cache_fsm::goal_acquired(void) const {
  if (foraging_acq_goal::ekBLOCK == acquisition_goal()) {
    return current_state() == ekST_WAIT_FOR_BLOCK_PICKUP;
  } else if (foraging_transport_goal::ekEXISTING_CACHE ==
             block_transport_goal()) {
    return current_state() == ekST_WAIT_FOR_BLOCK_DROP;
  }
  return false;
} /* goal_acquired() */

rtypes::type_uuid block_to_existing_cache_fsm::entity_acquired_id(void) const {
  if (foraging_acq_goal::ekBLOCK == acquisition_goal()) {
    return m_block_fsm.entity_acquired_id();
  } else if (foraging_transport_goal::ekEXISTING_CACHE ==
             block_transport_goal()) {
    return m_cache_fsm.entity_acquired_id();
  }
  return rtypes::constants::kNoUUID;
} /* entity_acquired_id() */

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
acquire_existing_cache_fsm cache_fsm_build(const fsm_ro_params* c_ro,
                                           const csfsm::fsm_params* c_no,
                                           rmath::rng* rng) {
  auto strategy_params = fstrategy::strategy_params{
    .fsm = c_no,
    .explore = &c_ro->strategy.caches.explore,
    .bsel_matrix = nullptr,
    .csel_matrix = c_ro->csel_matrix,
    .accessor = c_ro->accessor,
    .ledtaxis_target = carepr::light_type_index()[carepr::light_type_index::kCache]
  };

  auto strategy = fsexplore::cache_factory().create(
      c_ro->strategy.caches.explore.strategy, &strategy_params, rng);

  return acquire_existing_cache_fsm(c_ro,
                                    c_no,
                                    std::move(strategy),
                                    rng,
                                    false);
} /* cache_fsm_build() */

acquire_free_block_fsm block_fsm_build(const fsm_ro_params* c_ro,
                                       const csfsm::fsm_params* c_no,
                                       rmath::rng* rng) {
  auto strategy_params = fstrategy::strategy_params{
    .fsm = c_no,
    .explore = &c_ro->strategy.blocks.explore,
    .bsel_matrix = nullptr,
    .csel_matrix = nullptr,
    .accessor = c_ro->accessor,
    .ledtaxis_target = rutils::color()
  };
  auto strategy = fsexplore::block_factory().create(
      c_ro->strategy.blocks.explore.strategy, &strategy_params, rng);

  return acquire_free_block_fsm(c_ro, c_no, std::move(strategy), rng);
} /* block_fsm_build() */

NS_END(d1, controller, fordyca);
