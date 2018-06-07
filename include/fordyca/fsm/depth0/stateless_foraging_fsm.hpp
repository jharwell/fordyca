/**
 * @file stateless_foraging_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH0_STATELESS_FORAGING_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH0_STATELESS_FORAGING_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/explore_for_block_fsm.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/metrics/fsm/block_transport_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace state_machine = rcppsw::patterns::state_machine;
namespace visitor = rcppsw::patterns::visitor;
namespace params { struct fsm_params; }
namespace controller { class base_sensing_subsystem; class actuation_subsystem;}

NS_START(fsm, depth0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class stateless_foraging_fsm
 * @ingroup fsm depth0
 *
 * @brief The FSM for the most basic foraging definition: each robot executing
 * this FSM roams around randomly until it finds a block, and then brings the
 * block back to the nest, and drops it.
 */
class stateless_foraging_fsm : public base_foraging_fsm,
                               public metrics::fsm::goal_acquisition_metrics,
                               public metrics::fsm::block_transport_metrics,
                               public visitor::visitable_any<stateless_foraging_fsm> {
 public:
  stateless_foraging_fsm(const std::shared_ptr<rcppsw::er::server>& server,
                         const std::shared_ptr<controller::saa_subsystem>& saa);

  stateless_foraging_fsm(const stateless_foraging_fsm& fsm) = delete;
  stateless_foraging_fsm& operator=(const stateless_foraging_fsm& fsm) = delete;

  /* base FSM metrics */
  bool is_avoiding_collision(void) const override {
    return base_foraging_fsm::is_avoiding_collision();
  }

  /* goal acquisition metrics */
  goal_acquisition_metrics::goal_type goal(void) const override {
    return goal_acquisition_metrics::goal_type::kBlock;
  }
  bool is_exploring_for_goal(void) const override;
  bool is_vectoring_to_goal(void) const override { return false; }
  bool goal_acquired(void) const override;

  /* block transport metrics */
  bool is_transporting_to_nest(void) const override;
  bool is_transporting_to_cache(void) const override { return false; }

  /**
   * @brief (Re)-initialize the FSM.
   */
  void init(void) override;

  /**
   * @brief Run the FSM in its current state, without injecting an event.
   */
  void run(void);


 protected:
  enum fsm_states {
    ST_START, /* Initial state */
    ST_ACQUIRE_BLOCK,
    ST_TRANSPORT_TO_NEST,        /* Block found--bring it back to the nest */
    ST_LEAVING_NEST,          /* Block dropped in nest--time to go */
    ST_WAIT_FOR_BLOCK_PICKUP,
    ST_MAX_STATES
  };

  /* inherited states */
  HFSM_STATE_INHERIT(base_foraging_fsm, transport_to_nest,
                     state_machine::event_data);
  HFSM_STATE_INHERIT(base_foraging_fsm, leaving_nest,
                     state_machine::event_data);

  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_transport_to_nest);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_leaving_nest);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_wait_for_signal);

  /* stateless fsm states */
  HFSM_STATE_DECLARE(stateless_foraging_fsm, start, state_machine::event_data);
  HFSM_STATE_DECLARE_ND(stateless_foraging_fsm, acquire_block);
  HFSM_STATE_DECLARE(stateless_foraging_fsm, wait_for_block_pickup,
                     state_machine::event_data);

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
  argos::CRandom::CRNG* m_rng;
  explore_for_block_fsm m_explore_fsm;
  // clang-format on

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(depth0, controller, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH0_STATELESS_FORAGING_FSM_HPP_ */
