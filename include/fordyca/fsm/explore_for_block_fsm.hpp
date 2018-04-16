/**
 * @file explore_for_block_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_EXPLORE_FOR_BLOCK_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_EXPLORE_FOR_BLOCK_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include "fordyca/fsm/base_explore_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class explore_for_block_fsm
 * @ingroup fsm
 *
 * @brief Robots executing this task will roam around randomly looking for a
 * free block. Once they have found one, the FSM will signal that its task is
 * complete.
 */
class explore_for_block_fsm : public base_explore_fsm {
 public:
  enum fsm_states {
    ST_START,
    /**
     * Roaming around looking for a block.
     */
    ST_EXPLORE,
    /**
     * A block has been acquired.
     */
    ST_FINISHED,
    ST_MAX_STATES
  };

  explore_for_block_fsm(
      const std::shared_ptr<rcppsw::er::server>& server,
      const std::shared_ptr<controller::saa_subsystem>& saa);

  /* taskable overrides */
  bool task_finished(void) const override {
    return ST_FINISHED == current_state();
  }
  bool task_running(void) const override;
  void task_reset(void) override { init(); }

 private:
  /* inherited states */
  HFSM_ENTRY_INHERIT_ND(base_explore_fsm, entry_explore);

  /* exploration states */

  /**
   * @brief Starting/reset state for FSM. Has no purpose other than that.
   */
  HFSM_STATE_DECLARE_ND(explore_for_block_fsm, start);

  /**
   * @brief The main state for the explore FSM. Robots in this state maintain
   * their heading, looking for SOMETHING of interest, until they find it or
   * exceed the direction change threshold.
   */
  HFSM_STATE_DECLARE_ND(explore_for_block_fsm, explore);

  /**
   * @brief Once a block has been acquired, robots wait in this state until
   * reset by a higher level FSM.
   */
  HFSM_STATE_DECLARE_ND(explore_for_block_fsm, finished);

  /**
   * @brief Defines the state map for the FSM.
   *
   * Note that the order of the states in the map MUST match the order of the
   * states in \enum fsm_states, or things will not work correctly.
   */
  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
    return &mc_state_map[index];
  }

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_EXPLORE_FOR_BLOCK_FSM_HPP_ */
