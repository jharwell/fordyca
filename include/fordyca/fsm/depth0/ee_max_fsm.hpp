/**
 * @file ee_max_fsm.hpp
 *
 * @copyright 2018 Anthony Chen/John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH0_EE_MAX_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH0_EE_MAX_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/explore_for_goal_fsm.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/metrics/fsm/collision_metrics.hpp"
#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/controller/crw_controller.hpp"


/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace state_machine = rcppsw::patterns::state_machine;
namespace visitor = rcppsw::patterns::visitor;
namespace controller { class sensing_subsystem; class actuation_subsystem;}

NS_START(fsm, depth0);
using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using transport_goal_type = block_transporter::goal_type;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class ee_max_fsm
 * @ingroup fsm depth0
 *
 * @brief The FSM for the most performing an energy efficient foraging method:
 * each robot takes into consideration the battery left while foraging
 */
class ee_max_fsm : public base_foraging_fsm,
                               public er::client<ee_max_fsm>,
                               public metrics::fsm::goal_acquisition_metrics,
                               public block_transporter,
                               public visitor::visitable_any<ee_max_fsm> {
 public:
  explicit ee_max_fsm(controller::saa_subsystem* saa);

  ee_max_fsm(const ee_max_fsm& fsm) = delete;
  ee_max_fsm& operator=(const ee_max_fsm& fsm) = delete;

  /* collision metrics */
  FSM_WRAPPER_DECLAREC(bool, in_collision_avoidance);
  FSM_WRAPPER_DECLAREC(bool, entered_collision_avoidance);
  FSM_WRAPPER_DECLAREC(bool, exited_collision_avoidance);
  FSM_WRAPPER_DECLAREC(uint, collision_avoidance_duration);

  /* goal acquisition metrics */
  acquisition_goal_type acquisition_goal(void) const override;
  bool is_exploring_for_goal(void) const override;
  bool is_vectoring_to_goal(void) const override { return false; }
  bool goal_acquired(void) const override;

  /* block transportation */
  transport_goal_type block_transport_goal(void) const override;

  /**
   * @brief (Re)-initialize the FSM.
   */
  void init(void) override;

  /**
   * @brief Run the FSM in its current state, without injecting an event.
   */
  void run(void);

 private:
  bool block_detected(void) const;

  enum fsm_states {
    ST_START, /* Initial state */
    ST_ACQUIRE_BLOCK,
    ST_RETURN_TO_NEST,        /* Block found--bring it back to the nest */
    ST_LEAVING_NEST,          /* Block dropped in nest--time to go */
    ST_WAIT_FOR_BLOCK_PICKUP,
    ST_WAIT_FOR_BLOCK_DROP,
    ST_CHARGING,
    ST_MAX_STATES,
  };

  /* inherited states */
  HFSM_STATE_INHERIT(base_foraging_fsm, return_to_nest,
                     state_machine::event_data);
  HFSM_STATE_INHERIT(base_foraging_fsm, leaving_nest,
                     state_machine::event_data);

  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_return_to_nest);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_leaving_nest);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_wait_for_signal);

  /* ee_max fsm states */
  HFSM_STATE_DECLARE(ee_max_fsm, start, state_machine::event_data);
  HFSM_STATE_DECLARE_ND(ee_max_fsm, acquire_block);
  HFSM_STATE_DECLARE(ee_max_fsm, wait_for_block_pickup,
                     state_machine::event_data);
  HFSM_STATE_DECLARE(ee_max_fsm, wait_for_block_drop,
                     state_machine::event_data);
  HFSM_STATE_DECLARE_ND(ee_max_fsm, charging);


  /**
   * @brief Defines the state map for the FSM.
   *
   * Note that the order of the states in the map MUST match the order of the
   * states in \enum fsm_states, or things will not work correctly.
   */
  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
  return (&mc_state_map[index]);
  }

  // clang-format off
  explore_for_goal_fsm m_explore_fsm;
  // clang-format on

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(depth0, controller, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH0_ee_max_FSM_HPP_ */
