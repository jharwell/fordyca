/**
 * @file dpo_fsm.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH0_DPO_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH0_DPO_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/fsm/block_transporter.hpp"

#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/depth0/free_block_to_nest_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace ds { class dpo_store; }
namespace rta = rcppsw::ta;

NS_START(fsm, depth0);

using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using transport_goal_type = block_transporter::goal_type;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class dpo_fsm
 * @ingroup fsm depth0
 *
 * @brief The FSM for an unpartitioned foraging task. Each robot executing this
 * FSM will locate for a block (either a known block or via random exploration),
 * pickup the block and bring it all the way back to the nest.
 *
 * This FSM will only pickup free blocks. Once it has brought a block all the
 * way to the nest and dropped it in the nest, it will repeat the same sequence
 * (i.e. it loops indefinitely).
 */
class dpo_fsm : public base_foraging_fsm,
                     er::client<dpo_fsm>,
                     public metrics::fsm::goal_acquisition_metrics,
                     public block_transporter {
 public:
  dpo_fsm(const controller::block_sel_matrix* sel_matrix,
               controller::saa_subsystem* saa,
               ds::dpo_store* store);
  ~dpo_fsm(void) override = default;

  /* collision metrics */
  FSM_OVERRIDE_DECL(bool, in_collision_avoidance, const);
  FSM_OVERRIDE_DECL(bool, entered_collision_avoidance, const);
  FSM_OVERRIDE_DECL(bool, exited_collision_avoidance, const);
  FSM_OVERRIDE_DECL(uint, collision_avoidance_duration, const);

  /* goal acquisition metrics */
  FSM_OVERRIDE_DECL(bool, is_exploring_for_goal, const);
  FSM_OVERRIDE_DECL(bool, is_vectoring_to_goal, const);
  FSM_OVERRIDE_DECL(bool, goal_acquired, const);
  FSM_OVERRIDE_DECL(acquisition_goal_type, acquisition_goal, const);
  FSM_OVERRIDE_DECL(rmath::vector2u, acquisition_loc, const);

  /* block transportation */
  FSM_OVERRIDE_DECL(transport_goal_type, block_transport_goal, const);

  void init(void) override;

  /**
   * @brief Run the FSM in its current state, without injecting an event.
   */
  void run(void);

 protected:
  enum fsm_states {
    kST_START,
    kST_BLOCK_TO_NEST,     /* Find a block and bring it to the nest */
    kST_LEAVING_NEST,      /* Block dropped in nest--time to go */
    kST_MAX_STATES
  };

 private:
  /* inherited states */
  HFSM_STATE_INHERIT(base_foraging_fsm, leaving_nest,
                     rfsm::event_data);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_leaving_nest);

  /* foraging states */
  HFSM_STATE_DECLARE(dpo_fsm, start, rfsm::event_data);
  HFSM_STATE_DECLARE(dpo_fsm, block_to_nest, rfsm::event_data);

  /**
   * @brief Defines the state map for the FSM.
   *
   * Note that the order of the states in the map MUST match the order of the
   * states in \enum fsm_states, or things will not work correctly.
   */
  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
  return &mc_state_map[index];
  }

  /* clang-format off */
  free_block_to_nest_fsm m_block_fsm;
  /* clang-format on */

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, kST_MAX_STATES);
};

NS_END(depth0, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH0_DPO_FSM_HPP_ */
