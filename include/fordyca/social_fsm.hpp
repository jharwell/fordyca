/**
 * @file social_social_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_SOCIAL_FSM_HPP_
#define INCLUDE_FORDYCA_SOCIAL_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>
#include "rcppsw/patterns/state_machine/base_fsm.hpp"
#include "fordyca/fordyca_params.hpp"
#include "fordyca/sensor_manager.hpp"
#include "fordyca/actuator_manager.hpp"
#include "rcsw/common/common.h"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace fsm = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class social_fsm : public fsm::base_fsm {
 public:
  enum fsm_states {
    ST_REST,
    ST_EXPLORE,
    ST_EXPLORE_SUCCESS,
    ST_EXPLORE_FAIL,
    ST_RETURN_TO_NEST,
    ST_SEARCH_FOR_SPOT_IN_NEST,
    ST_COLLISION_AVOIDANCE,
    ST_MAX_STATES
  };

  struct collision_event_data : public fsm::event_data {
    enum fsm_states last_state;

    collision_event_data(void) : last_state(ST_REST) {}
  };

  social_fsm(const struct social_fsm_params* params,
             sensor_manager* const sensors,
             actuator_manager* const actuators);

  bool is_resting(void) { return current_state() == ST_REST; }
  void reset(void);
  void event_explore(void);

 private:
  /**
   * @brief This structure holds data about food collecting by the robots
   */
  struct food_data {
    food_data(void) : has_item(false), curr_item_idx(-1), cum_items(0) {}
    void Reset(void);

    bool has_item;      // true when the robot is carrying a food item
    int curr_item_idx;    // the index of the current food item in the array of available food items
    size_t cum_items; // the total number of food items carried by this robot during the experiment

  };

  struct fsm_state {
    fsm_state(void) :
        rest_to_explore_prob(),
        explore_to_rest_prob(),
        time_rested(0),
        time_exploring_unsuccessfully(0),
        time_search_for_place_in_nest(0),
        food_data() {}

    /* Current probability to switch from resting to exploring */
    argos::Real rest_to_explore_prob;
    /* Current probability to switch from exploring to resting */
    argos::Real explore_to_rest_prob;
    /* The number of steps in resting state */
    size_t time_rested;
    size_t time_exploring_unsuccessfully;
    size_t time_search_for_place_in_nest;
    struct food_data food_data;
  };

  STATE_DECLARE(social_fsm, rest, fsm::no_event_data);
  STATE_DECLARE(social_fsm, explore, fsm::no_event_data);
  STATE_DECLARE(social_fsm, explore_success, fsm::no_event_data);
  STATE_DECLARE(social_fsm, explore_fail, fsm::no_event_data);
  STATE_DECLARE(social_fsm, return_to_nest, fsm::no_event_data);
  STATE_DECLARE(social_fsm, search_for_spot_in_nest, fsm::no_event_data);
  STATE_DECLARE(social_fsm, collision_avoidance, struct collision_event_data);

  EXIT_DECLARE(social_fsm, exit_explore);
  EXIT_DECLARE(social_fsm, exit_rest);
  EXIT_DECLARE(social_fsm, exit_collision_avoidance);
  EXIT_DECLARE(social_fsm, exit_search_for_spot_in_nest);

  ENTRY_DECLARE(social_fsm, entry_explore, fsm::no_event_data);
  ENTRY_DECLARE(social_fsm, entry_rest, fsm::no_event_data);
  ENTRY_DECLARE(social_fsm, entry_collision_avoidance, fsm::no_event_data);
  ENTRY_DECLARE(social_fsm, entry_search_for_spot_in_nest, fsm::no_event_data);

  virtual const fsm::state_map_ex_row* state_map_ex() {
    static const fsm::state_map_ex_row kSTATE_MAP[] = {
      {&rest, NULL, &entry_rest, NULL},
      {&explore, NULL, &entry_explore, &exit_explore},
      {&explore_success, NULL, NULL, NULL},
      {&explore_fail, NULL, NULL, NULL},
      {&return_to_nest, NULL, NULL , NULL},
      {&search_for_spot_in_nest, NULL, NULL, NULL},
      {&collision_avoidance, NULL, NULL, NULL}
    };
    static_assert((sizeof(kSTATE_MAP)/sizeof(struct fsm::state_map_ex_row)) == ST_MAX_STATES,
                  "state map does not cover all states");
    return kSTATE_MAP;
  }

  social_fsm(const social_fsm& fsm) = delete;
  social_fsm& operator=(const social_fsm& fsm) = delete;

  /* data members */
  enum last_exploration_result {
    LAST_EXPLORATION_NONE = 0,    // nothing to report
    LAST_EXPLORATION_SUCCESSFUL,  // the last exploration resulted in a food item found
    LAST_EXPLORATION_UNSUCCESSFUL // no food found in the last exploration
  };

  enum last_exploration_result m_last_explore_res;
  argos::CRandom::CRNG* m_rng;
  argos::CRange<argos::Real> m_prob_range;
  struct fsm_state m_state;
  std::shared_ptr<const struct social_fsm_params> mc_params;
  std::shared_ptr<sensor_manager> m_sensors;
  std::shared_ptr<actuator_manager> m_actuators;
};

NS_END(fordyca);

#endif /* INCLUDE_FORDYCA_SOCIAL_FSM_HPP_ */
