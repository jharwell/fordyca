/**
 * \file block_to_new_cache_fsm.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/acquire_free_block_fsm.hpp"
#include "fordyca/fsm/d2/acquire_new_cache_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, d2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_to_new_cache_fsm
 * \ingroup fsm d2
 *
 * \brief The FSM for the block-to-new-cache subtask.
 *
 * Each robot executing this FSM will locate a free block (either a known block
 * or via exploration), pickup the block and bring it to the best new cache it
 * knows about. Once it has done that it will signal that its task is complete.
 */
class block_to_new_cache_fsm final : public block_to_goal_fsm {
 public:
  block_to_new_cache_fsm(const fsm_ro_params* c_ro,
                         const csfsm::fsm_params* c_no,
                         cffsm::strategy_set strategies,
                         rmath::rng* rng);
  ~block_to_new_cache_fsm(void) override = default;

  block_to_new_cache_fsm(const block_to_new_cache_fsm& fsm) = delete;
  block_to_new_cache_fsm& operator=(const block_to_new_cache_fsm& fsm) = delete;

  /* goal acquisition metrics */
  csmetrics::goal_acq_metrics::goal_type acquisition_goal(void) const override RCPPSW_PURE;
  rtypes::type_uuid entity_acquired_id(void) const override;

  /* block transportation */
  foraging_transport_goal block_transport_goal(void) const override RCPPSW_PURE;
  bool is_phototaxiing_to_goal(bool) const override { return false; }

  /* clang-format off */
  acquire_new_cache_fsm  m_cache_fsm;
  acquire_free_block_fsm m_block_fsm;
  /* clang-format on */
};

NS_END(d2, fsm, fordyca);
