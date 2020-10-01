/**
 * \file acquire_existing_cache_fsm.cpp
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
#include "fordyca/fsm/d1/block_to_existing_cache_fsm.hpp"

#include "cosm/arena/repr/light_type_index.hpp"

#include "fordyca/fsm/expstrat/block_factory.hpp"
#include "fordyca/fsm/expstrat/cache_factory.hpp"
#include "fordyca/fsm/expstrat/foraging_expstrat.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, d1);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_to_existing_cache_fsm::block_to_existing_cache_fsm(
    const fsm_ro_params* const c_params,
    crfootbot::footbot_saa_subsystem* saa,
    rmath::rng* rng)
    : block_to_goal_fsm(&m_cache_fsm, &m_block_fsm, saa, rng),
      m_cache_fsm(
          c_params,
          saa,
          expstrat::cache_factory().create(
              c_params->exp_config.cache_strategy,
              std::make_unique<expstrat::foraging_expstrat::params>(
                  saa,
                  nullptr,
                  c_params->csel_matrix,
                  c_params->store,
                  carepr::light_type_index()[carepr::light_type_index::kCache])
                  .get(),
              rng),
          rng,
          false),
      m_block_fsm(c_params,
                  saa,
                  expstrat::block_factory().create(
                      c_params->exp_config.block_strategy,
                      std::make_unique<expstrat::foraging_expstrat::params>(
                          saa,
                          nullptr,
                          nullptr,
                          c_params->store,
                          rutils::color())
                          .get(),
                      rng),
                  rng) {}

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
csmetrics::goal_acq_metrics::goal_type block_to_existing_cache_fsm::
    acquisition_goal(void) const {
  if (ekST_ACQUIRE_BLOCK == current_state() ||
      ekST_WAIT_FOR_BLOCK_PICKUP == current_state()) {
    return fsm::to_goal_type(foraging_acq_goal::ekBLOCK);
  } else if (ekST_TRANSPORT_TO_GOAL == current_state() ||
             ekST_WAIT_FOR_BLOCK_DROP == current_state()) {
    return fsm::to_goal_type(foraging_acq_goal::ekEXISTING_CACHE);
  }
  return fsm::to_goal_type(foraging_acq_goal::ekNONE);
} /* acquisition_goal() */

foraging_transport_goal block_to_existing_cache_fsm::block_transport_goal(
    void) const {
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

NS_END(d1, controller, fordyca);
