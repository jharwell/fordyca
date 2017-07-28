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

#ifndef INCLUDE_FORDYCA_SOCIAL_SOCIAL_FSM_HPP_
#define INCLUDE_FORDYCA_SOCIAL_SOCIAL_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>
#include "rcppsw/patterns/state_machine/base_fsm.hpp"
#include "fordyca/fordyca_params.hpp"
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
  social_fsm(const struct social_fsm_config& config) :
      fsm::base_fsm(ST_MAX_STATES),
      config_(config) {}

  struct collision_event_data : public fsm::event_data {
    argos::CVector2 vector;
  };

 private:
  struct fsm_state {
    /* Current probability to switch from resting to exploring */
    argos::Real rest_to_explore_prob;
    /* Current probability to switch from exploring to resting */
    argos::Real explore_to_rest_prob;
    /* The number of steps in resting state */
    size_t time_rested;
    size_t time_exploring_unsuccessfully;
  };

  void event_explore(void);

  STATE_DECLARE(social_fsm, rest, fsm::no_event_data);
  STATE_DECLARE(social_fsm, explore, fsm::no_event_data);
  STATE_DECLARE(social_fsm, explore_success, fsm::no_event_data);
  STATE_DECLARE(social_fsm, explore_fail, fsm::no_event_data);
  STATE_DECLARE(social_fsm, return_to_nest, fsm::no_event_data);
  STATE_DECLARE(social_fsm, collision_avoidance, fsm::no_event_data);
  STATE_DECLARE(social_fsm, in_nest, fsm::no_event_data);

  GUARD_DECLARE(social_fsm, guard_collision_avoidance, struct collision_event_data);
  GUARD_DECLARE(social_fsm, guard_return_to_nest, fsm::no_event_data);
  GUARD_DECLARE(social_fsm, guard_explore, fsm::no_event_data);

  EXIT_DECLARE(social_fsm, exit_explore);
  EXIT_DECLARE(social_fsm, exit_rest);
  EXIT_DECLARE(social_fsm, exit_collision_avoidance);

  ENTRY_DECLARE(social_fsm, entry_explore, fsm::no_event_data);
  ENTRY_DECLARE(social_fsm, entry_rest, fsm::no_event_data);

  virtual const fsm::state_map_ex_row* state_map_ex() {
    static const fsm::state_map_ex_row kSTATE_MAP[] = {
      {&rest, NULL, &entry_rest, NULL},
      {&explore, NULL, &entry_explore, &exit_explore},
      {&explore_success, NULL, NULL, NULL},
      {&explore_fail, NULL, NULL, NULL},
      {&return_to_nest, NULL, NULL , NULL},
      {&collision_avoidance, NULL, NULL, NULL}
    };
    static_assert((sizeof(kSTATE_MAP)/sizeof(struct fsm::state_map_ex_row)) == ST_MAX_STATES,
                  "state map does not cover all states");
    return kSTATE_MAP;
  }

  /* data members */
  enum states {
    ST_REST,
    ST_EXPLORE,
    ST_EXPLORE_SUCCESS,
    ST_EXPLORE_FAIL,
    ST_RETURN_TO_NEST,
    ST_COLLISION_AVOIDANCE,
    ST_MAX_STATES
  };
  const struct social_fsm_config config_;
  struct fsm_state state_;
};

NS_END(fordyca);

#endif /* INCLUDE_FORDYCA_SOCIAL_FSM_HPP_ */
