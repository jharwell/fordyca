/**
 * @file vector_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_VECTOR_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_VECTOR_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/tasks/argument.hpp"
#include "rcppsw/task_allocation/taskable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace state_machine = rcppsw::patterns::state_machine;
namespace task_allocation = rcppsw::task_allocation;

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class vector_fsm
 * @ingroup fsm
 *
 * @brief An FSM used to send a robot to a particular ABSOLUTE location in the
 * arena.
 *
 * Vectoring is controlled by two PID loops: one for angle between robot heading
 * and the heading to the goal, and one for distance of robot to the goal.
 *
 * Arrival tolerance can be specified differently for blocks and caches, which
 * is necessary to avoid false positives in the case of blocks, and also to
 * avoid multiple robots all trying to drive to the center of the cache to
 * "arrive" at it.
 */
class vector_fsm : public base_foraging_fsm, public task_allocation::taskable {
 public:
  /**
   * @brief The tolerance within which a robot's location has to be in order to
   * be considered having arrived at the specified block's location.
   */
  constexpr static double kBLOCK_ARRIVAL_TOL = 0.02;

  /**
   * @brief The tolerance within which a robot's location has to be in order to
   * be considered having arrived at the specified cache's location.
   */
  constexpr static double kCACHE_ARRIVAL_TOL = 0.3;

  /**
   * @brief The tolerance within which a robot's location has to be in order to
   * be considered to have arrived at the specified cache site.
   */
  constexpr static double kCACHE_SITE_ARRIVAL_TOL = 0.02;

  vector_fsm(const std::shared_ptr<rcppsw::er::server>& server,
             controller::saa_subsystem* saa);

  vector_fsm(const vector_fsm& fsm) = delete;
  vector_fsm& operator=(const vector_fsm& fsm) = delete;

  /* taskable overrides */
  void task_reset(void) override { init(); }
  bool task_running(void) const override {
    return current_state() != ST_START && current_state() != ST_ARRIVED;
  }
  void task_execute(void) override;
  void task_start(
      const rcppsw::task_allocation::taskable_argument* c_arg) override;
  bool task_finished(void) const override {
    return current_state() == ST_ARRIVED;
  }

  /**
   * @brief Initialize/re-initialize the vector_fsm fsm. After arriving at a
   * goal, this function must be called before vectoring to a new goal will
   * work.
   */
  void init(void) override;

  bool is_avoiding_collision(void) const override {
    return ST_COLLISION_AVOIDANCE == current_state();
  }

 protected:
  enum fsm_states {
    ST_START,
    /**
     * Vectoring toward the target.
     */
    ST_VECTOR,

    /**
     * Avoiding an obstacle nearby to the robot's current location.
     */
    ST_COLLISION_AVOIDANCE,

    /**
     * Recovering from frequent collision avoidance by driving AWAY from the
     * site of the most recent collision in a random direction for a set number
     * of timesteps. This is intended to help prevent robot's from wasting lots
     * of time butting heads when they are traveling in opposite/spatially
     * conflicting directions.
     */
    ST_COLLISION_RECOVERY,

    /**
     * We have been colliding too frequently--time to change things up and
     * hopefully move away from the problem location.
     */
    ST_NEW_DIRECTION,

    /**
     * We have arrived at the specified location within tolerance.
     */
    ST_ARRIVED,
    ST_MAX_STATES
  };

 private:
  /**
   * @brief A structure containing all the information needed for the controller
   * to tell the FSM where to travel to next.
   */
  struct goal_data : public state_machine::event_data {
    goal_data(argos::CVector2 loc_, double tolerance_)
        : tolerance(tolerance_), loc(loc_) {}
    goal_data(void) : loc() {}

    double tolerance{0.0};
    argos::CVector2 loc;
  };

  struct fsm_state {
    uint m_collision_rec_count{0};
    uint last_collision_time{0};
  };

  /**
   * @brief The # of timesteps according to collision recovery. This is mainly
   * to ensure that you do not repeatedly get 2 robots butting heads as they try
   * to travel to opposite goals.
   */
  constexpr static uint kCOLLISION_RECOVERY_TIME = 10;

  /**
   * @brief If a robotics sees a threatening obstacle more than twice in this
   * interval, it is considered to be colliding too frequently, and will enter
   * collision recovery.
   */
  constexpr static uint kFREQ_COLLISION_THRESH = 300;

  /**
   * @brief Calculates the relative vector from the robot to the current goal.
   *
   * @param goal The current goal.
   *
   * @return The vector, specified with the tail at the robot and the head
   * pointing towards the goal.
   */
  argos::CVector2 calc_vector_to_goal(const argos::CVector2& goal);

  /* inherited states */
  HFSM_STATE_INHERIT(base_foraging_fsm, new_direction, state_machine::event_data);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_new_direction);

  /* vector states */
  HFSM_STATE_DECLARE_ND(vector_fsm, start);
  HFSM_STATE_DECLARE(vector_fsm, vector, state_machine::event_data);
  HFSM_STATE_DECLARE_ND(vector_fsm, collision_avoidance);
  HFSM_STATE_DECLARE_ND(vector_fsm, collision_recovery);
  HFSM_STATE_DECLARE(vector_fsm, arrived, struct goal_data);

  FSM_ENTRY_DECLARE_ND(vector_fsm, entry_vector);
  FSM_ENTRY_DECLARE_ND(vector_fsm, entry_collision_avoidance);
  FSM_ENTRY_DECLARE_ND(vector_fsm, entry_collision_recovery);

  /**
   * @brief Defines the state map for the FSM.
   *
   * Note that the order of the states in the map MUST match the order of the
   * states in \enum fsm_states, or things will not work correctly.
   *
   * Note also that all robots will share the SAME state map in memory, so you
   * cannot change the parent of any statein this FSM for only SOME other
   * objects. But that should not be necessary, as it is taskable.
   */
  FSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
    FSM_DEFINE_STATE_MAP(state_map_ex, kSTATE_MAP){
        FSM_STATE_MAP_ENTRY_EX_ALL(&start, nullptr, nullptr, nullptr),
        FSM_STATE_MAP_ENTRY_EX_ALL(&vector, nullptr, &entry_vector, nullptr),
        FSM_STATE_MAP_ENTRY_EX_ALL(
            &collision_avoidance, nullptr, &entry_collision_avoidance, nullptr),
        FSM_STATE_MAP_ENTRY_EX_ALL(
            &collision_recovery, nullptr, &entry_collision_recovery, nullptr),
        FSM_STATE_MAP_ENTRY_EX_ALL(
            &new_direction, nullptr, &entry_new_direction, nullptr),
        FSM_STATE_MAP_ENTRY_EX_ALL(&arrived, nullptr, nullptr, nullptr)};
    return &kSTATE_MAP[index];
  }

  // clang-format off
  struct fsm_state m_state;
  struct goal_data m_goal_data;
  // clang-format on
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_VECTOR_FSM_HPP_ */
