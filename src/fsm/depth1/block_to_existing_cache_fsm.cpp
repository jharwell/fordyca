/**
 * @file acquire_existing_cache_fsm.cpp
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
#include "fordyca/fsm/depth1/block_to_existing_cache_fsm.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/fsm/expstrat/block_factory.hpp"
#include "fordyca/fsm/expstrat/cache_factory.hpp"
#include "fordyca/support/light_type_index.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth1);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_to_existing_cache_fsm::block_to_existing_cache_fsm(
    const params* const c_params)
    : block_to_goal_fsm(&m_cache_fsm, &m_block_fsm, c_params->saa),
      m_cache_fsm(
          c_params->csel_matrix,
          c_params->saa,
          c_params->store,
          expstrat::cache_factory().create(
              c_params->exp_config.cache_strategy,
              std::make_unique<expstrat::base_expstrat::params>(
                  c_params->csel_matrix,
                  c_params->saa,
                  c_params->store,
                  support::light_type_index()[support::light_type_index::kCache])
                  .get()),
          false),
      m_block_fsm(
          c_params->bsel_matrix,
          c_params->saa,
          c_params->store,
          expstrat::block_factory().create(
              c_params->exp_config.block_strategy,
              std::make_unique<expstrat::base_expstrat::params>(nullptr,
                                                                c_params->saa,
                                                                c_params->store)
                  .get())) {}

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
acq_goal_type block_to_existing_cache_fsm::acquisition_goal(void) const {
  if (ekST_ACQUIRE_BLOCK == current_state() ||
      ekST_WAIT_FOR_BLOCK_PICKUP == current_state()) {
    return acq_goal_type::ekBLOCK;
  } else if (ekST_TRANSPORT_TO_GOAL == current_state() ||
             ekST_WAIT_FOR_BLOCK_DROP == current_state()) {
    return acq_goal_type::ekEXISTING_CACHE;
  }
  return acq_goal_type::ekNONE;
} /* acquisition_goal() */

transport_goal_type block_to_existing_cache_fsm::block_transport_goal(void) const {
  if (ekST_TRANSPORT_TO_GOAL == current_state() ||
      ekST_WAIT_FOR_BLOCK_DROP == current_state()) {
    return transport_goal_type::ekEXISTING_CACHE;
  }
  return transport_goal_type::ekNONE;
} /* acquisition_goal() */

bool block_to_existing_cache_fsm::goal_acquired(void) const {
  if (acq_goal_type::ekBLOCK == acquisition_goal()) {
    return current_state() == ekST_WAIT_FOR_BLOCK_PICKUP;
  } else if (transport_goal_type::ekEXISTING_CACHE == block_transport_goal()) {
    return current_state() == ekST_WAIT_FOR_BLOCK_DROP;
  }
  return false;
} /* goal_acquired() */

NS_END(depth1, controller, fordyca);
