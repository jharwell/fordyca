/**
 * @file random_foraging_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_RANDOM_FORAGING_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_RANDOM_FORAGING_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/explore_fsm.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace state_machine = rcppsw::patterns::state_machine;
namespace visitor = rcppsw::patterns::visitor;

namespace params {
struct fsm_params;
} /* namespace params */

namespace controller {
class sensor_manager;
class actuator_manager;
} /* namespace controller */
NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class random_foraging_fsm
 *
 * @brief The FSM for the most basic foraging definition: each robot executing
 * this FSM roams around randomly until it finds a block, and then brings the
 * block back to the nest and repeat.
 */
class random_foraging_fsm : public base_foraging_fsm,
                            public visitor::visitable_any<random_foraging_fsm> {
 public:
  random_foraging_fsm(const struct params::fsm_params* params,
                      std::shared_ptr<rcppsw::common::er_server> server,
                      std::shared_ptr<controller::sensor_manager> sensors,
                      std::shared_ptr<controller::actuator_manager> actuators);

  /**
   * @brief If \c TRUE the robot is roaming around looking for a block.
   */
  virtual bool is_exploring(void) const;

  /**
   * @brief (Re)-initialize the FSM.
   */
  void init(void) override;

  /**
   * @brief Run the FSM in its current state, without injecting an event.
   */
  void run(void);

  /**
   * @brief If \c TRUE, the robot is returning to the nest, probably after having
   * successfully picked up a block.
   */
  bool is_transporting_to_nest(void) const {
    return current_state() == ST_RETURN_TO_NEST;
  }

  /**
   * @brief If TRUE, the robot is currently engaged in collision avoidance.
   */
  bool is_avoiding_collision(void) const {
    return m_explore_fsm.is_avoiding_collision();
  }

 protected:
  enum fsm_states {
    ST_START, /* Initial state */
    ST_ACQUIRE_BLOCK,
    ST_RETURN_TO_NEST,        /* Block found--bring it back to the nest */
    ST_LEAVING_NEST,          /* Block dropped in nest--time to go */
    ST_COLLISION_AVOIDANCE,
    ST_MAX_STATES
  };

  /* inherited states */
  HFSM_STATE_INHERIT(base_foraging_fsm, return_to_nest,
                     state_machine::event_data);
  HFSM_STATE_INHERIT(base_foraging_fsm, leaving_nest,
                     state_machine::event_data);
  HFSM_STATE_INHERIT_ND(base_foraging_fsm, collision_avoidance);

  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_return_to_nest);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_leaving_nest);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_collision_avoidance);

  /* random foraging fsm states */
  HFSM_STATE_DECLARE(random_foraging_fsm, start, state_machine::event_data);
  HFSM_STATE_DECLARE(random_foraging_fsm, acquire_block,
                     state_machine::event_data);

  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
  return (&mc_state_map[index]);
  }

  random_foraging_fsm(const random_foraging_fsm& fsm) = delete;
  random_foraging_fsm& operator=(const random_foraging_fsm& fsm) = delete;

  argos::CRandom::CRNG* m_rng;
  explore_fsm m_explore_fsm;
  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_RANDOM_FORAGING_FSM_HPP_ */
