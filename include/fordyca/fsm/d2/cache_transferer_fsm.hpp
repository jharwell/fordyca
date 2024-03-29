/**
 * \file cache_transferer_fsm.hpp
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
#include "fordyca/fsm/acquire_existing_cache_fsm.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace ds { class dpo_store; }

NS_START(fsm, d2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_transferer_fsm
 * \ingroup fsm d2
 *
 * \brief The FSM for a cache transferer task. Each robot executing this FSM
 * will acquire a cache (either a known cache or via exploration), pickup a
 * block from it and then bring it to ANOTHER cache (either a known cache or one
 * found via exploration) and drop it.
 */
class cache_transferer_fsm final : public block_to_goal_fsm {
 public:
  cache_transferer_fsm(const fsm_ro_params* c_ro,
                         const csfsm::fsm_params* c_no,
                       cffsm::strategy_set strategies,
                       rmath::rng* rng);
  ~cache_transferer_fsm(void) override = default;

  cache_transferer_fsm(const cache_transferer_fsm&) = delete;
  cache_transferer_fsm& operator=(const cache_transferer_fsm&) = delete;

  /* goal acquisition metrics */
  csmetrics::goal_acq_metrics::goal_type acquisition_goal(void) const override RCPPSW_PURE;
  rtypes::type_uuid entity_acquired_id(void) const override;

  /* block transportation */
  foraging_transport_goal block_transport_goal(void) const override RCPPSW_PURE;
  bool is_phototaxiing_to_goal(bool) const override { return false; }

  bool is_acquiring_dest_cache(void) const RCPPSW_PURE;
  bool is_acquiring_src_cache(void) const RCPPSW_PURE;

 private:
  /* clang-format off */
  acquire_existing_cache_fsm m_src_cache_fsm;
  acquire_existing_cache_fsm m_dest_cache_fsm;
  /* clang-format on */
};

NS_END(d2, fsm, fordyca);
