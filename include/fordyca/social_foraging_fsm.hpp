/**
 * @file social_foraging_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_SOCIAL_FORAGING_FSM_HPP_
#define INCLUDE_FORDYCA_SOCIAL_FORAGING_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/state_machine/base_fsm.hpp"
#include "rcsw/common/common.h"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace fsm = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class social_foraging_fsm : public fsm::base_fsm {
 public:
  /* constructors */
  social_foraging_fsm();

  /* member functions */


 private:
  /* member functions */

  /* data members */
  enum states {
    ST_REST,
    ST_EXPLORE,
    ST_EXPLORE_SUCCESS,
    ST_EXPLORE_FAIL,
    ST_RETURN_TO_NEST,
    ST_COLLISION_AVOIDANCE,
    ST_IN_NEST,
    ST_MAX_STATES
  };

  STATE_DECLARE(social_foraging_fsm, rest, fsm::no_event_data);
  STATE_DECLARE(social_foraging_fsm, explore, fsm::no_event_data);
  STATE_DECLARE(social_foraging_fsm, explore_success, fsm::no_event_data);
  STATE_DECLARE(social_foraging_fsm, explore_fail, fsm::no_event_data);
  STATE_DECLARE(social_foraging_fsm, return_to_nest, fsm::no_event_data);
  STATE_DECLARE(social_foraging_fsm, collision_avoidance, fsm::no_event_data);
  STATE_DECLARE(social_foraging_fsm, in_nest, fsm::no_event_data);

  GUARD_DECLARE(social_foraging_fsm, guard_collision_avoidance, collision_event_data);
  GUARD_DECLARE(social_foraging_fsm, guard_return_to_nest, fsm::no_event_data);
  GUARD_DECLARE(social_foraging_fsm, guard_explore, fsm::no_event_data);
  GUARD_DECLARE(social_foraging_fsm, guard_rest, fsm::no_event_data);

  EXIT_DECLARE(social_foraging_fsm, exit_explore);
  EXIT_DECLARE(social_foraging_fsm, exit_rest);
  EXIT_DECLARE(social_foraging_fsm, exit_collision_avoidance);

 private:
  virtual const fsm::state_map_ex_row* state_map_ex() {
    static const fsm::state_map_ex_row kSTATE_MAP[] = {
      {&rest},
      {&explore, NULL, NULL, &exit_explore},
      {&explore_success},
      {&explore_fail},
      {&return_to_nest, NULL, NULL , NULL},
      {&collision_avoidance, guard_collision_avoidance, NULL}
    };
    static_assert((sizeof(kSTATE_MAP)/sizeof(struct fsm::state_map_ex_row)) == ST_MAX_STATES,
                  "state map does not cover all states");
    return kSTATE_MAP;
  }
};

NS_END(fordyca);

#endif /* INCLUDE_FORDYCA_SOCIAL_FORAGING_FSM_HPP_ */
