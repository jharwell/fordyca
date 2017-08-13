/**
 * @file foraging_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_FORAGING_FSM_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_FORAGING_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>
#include "rcppsw/patterns/state_machine/simple_fsm.hpp"
#include "fordyca/params/params.hpp"
#include "fordyca/controller/sensor_manager.hpp"
#include "fordyca/controller/actuator_manager.hpp"
#include "rcsw/common/common.h"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
namespace fsm = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class foraging_fsm : public fsm::simple_fsm {
 public:
  enum fsm_states {
    ST_EXPLORE,
    ST_EXPLORE_SUCCESS,
    ST_EXPLORE_FAIL,
    ST_RETURN_TO_NEST,
    ST_LEAVING_NEST,
    ST_COLLISION_AVOIDANCE,
    ST_MAX_STATES
  };

  foraging_fsm(const struct foraging_fsm_params* params,
             std::shared_ptr<rcppsw::common::er_server> server,
             std::shared_ptr<sensor_manager> sensors,
             std::shared_ptr<actuator_manager> actuators);
  bool is_exploring(void) {return current_state() == ST_EXPLORE; }
  bool is_returning(void) {return current_state() == ST_RETURN_TO_NEST; }
  bool is_avoiding_collision(void) {return current_state() == ST_COLLISION_AVOIDANCE; }
  void init(void);
  void event_explore(void);
  void event_continue(void);
  void event_block_found(void);

  void run(void) { event_continue(); }

 private:
  /**
   * @brief This structure holds data about block collecting by the robots
   */
  struct block_data {
    block_data(void) : has_item(false), curr_item_idx(-1), cum_items(0) {}
    void Reset(void);

    bool has_item;      // true when the robot is carrying a block item
    int curr_item_idx;    // the index of the current block item in the array of available block items
    size_t cum_items; // the total number of block items carried by this robot during the experiment
  };

  struct fsm_state {
    fsm_state(void) :
        time_exploring_unsuccessfully(0),
        block_data() {}

    size_t time_exploring_unsuccessfully;
    struct block_data block_data;
  };

  FSM_STATE_DECLARE(foraging_fsm, explore, fsm::no_event_data);
  FSM_STATE_DECLARE(foraging_fsm, explore_success, fsm::no_event_data);
  FSM_STATE_DECLARE(foraging_fsm, explore_fail, fsm::no_event_data);
  FSM_STATE_DECLARE(foraging_fsm, return_to_nest, fsm::no_event_data);
  FSM_STATE_DECLARE(foraging_fsm, leaving_nest, fsm::no_event_data);
  FSM_STATE_DECLARE(foraging_fsm, collision_avoidance, fsm::no_event_data);

  FSM_ENTRY_DECLARE(foraging_fsm, entry_explore, fsm::no_event_data);
  FSM_ENTRY_DECLARE(foraging_fsm, entry_collision_avoidance,
                    fsm::no_event_data);
  FSM_ENTRY_DECLARE(foraging_fsm, entry_leaving_nest, fsm::no_event_data);

  FSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex) {
  FSM_DEFINE_STATE_MAP_EX(state_map_ex, kSTATE_MAP) {
        FSM_STATE_MAP_ENTRY_EX_ALL(&explore, NULL, &entry_explore, NULL),
        FSM_STATE_MAP_ENTRY_EX_ALL(&explore_success, NULL, NULL, NULL),
        FSM_STATE_MAP_ENTRY_EX_ALL(&explore_fail, NULL, NULL, NULL),
        FSM_STATE_MAP_ENTRY_EX_ALL(&return_to_nest, NULL, NULL, NULL),
        FSM_STATE_MAP_ENTRY_EX_ALL(&leaving_nest, NULL,
                                   &entry_leaving_nest, NULL),
        FSM_STATE_MAP_ENTRY_EX_ALL(&collision_avoidance, NULL,
                                   &entry_collision_avoidance, NULL),
    };
  FSM_VERIFY_STATE_MAP(state_map_ex, kSTATE_MAP);
    return &kSTATE_MAP[0];
  }

  foraging_fsm(const foraging_fsm& fsm) = delete;
  foraging_fsm& operator=(const foraging_fsm& fsm) = delete;

  /* data members */
  enum last_exploration_result {
    LAST_EXPLORATION_NONE = 0,    // nothing to report
    LAST_EXPLORATION_SUCCESSFUL,  // the last exploration resulted in a block item found
    LAST_EXPLORATION_UNSUCCESSFUL // no block found in the last exploration
  };

  enum last_exploration_result m_last_explore_res;
  argos::CRandom::CRNG* m_rng;
  argos::CRange<argos::Real> m_prob_range;
  struct fsm_state m_state;
  std::shared_ptr<const struct foraging_fsm_params> mc_params;
  std::shared_ptr<sensor_manager> m_sensors;
  std::shared_ptr<actuator_manager> m_actuators;
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_FORAGING_FSM_HPP_ */
