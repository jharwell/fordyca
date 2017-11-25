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
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>
#include "rcppsw/task_allocation/polled_simple_fsm.hpp"
#include "rcppsw/control/pid_loop.hpp"
#include "fordyca/tasks/argument.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace controller {
namespace depth0 { class foraging_sensors; }
class actuator_manager;
} /* namespace controller */

namespace state_machine = rcppsw::patterns::state_machine;
NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class vector_fsm : public rcppsw::task_allocation::polled_simple_fsm {
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
constexpr static double kCACHE_ARRIVAL_TOL = 0.2;

  vector_fsm(uint frequent_collision_thresh,
             std::shared_ptr<rcppsw::er::server> server,
             std::shared_ptr<controller::depth0::foraging_sensors> sensors,
             std::shared_ptr<controller::actuator_manager> actuators);

  /* taskable overrides */

  void task_reset(void) override { init(); }
  bool task_running(void) const override {
    return current_state() != ST_START && current_state() != ST_ARRIVED;
  }

  void task_start(const rcppsw::task_allocation::taskable_argument* const arg) override;
  bool task_finished(void) const override { return current_state() == ST_ARRIVED; }

  /**
   * @brief Initialize/re-initialize the vector_fsm fsm. After arriving at a
   * goal, this function must be called before vectoring to a new goal will work.
   */
  void init(void) override;


  bool is_avoiding_collision(void) const {
    return ST_COLLISION_AVOIDANCE == current_state() ||
        ST_COLLISION_AVOIDANCE == current_state();
  }

 protected:
  enum fsm_states {
    ST_START,
    ST_VECTOR,
    ST_COLLISION_AVOIDANCE,
    ST_COLLISION_RECOVERY,
    ST_ARRIVED,
    ST_MAX_STATES
  };

 private:
  /* types */

  /**
   * @brief A structure containing all the information needed for the controller
   * to tell the FSM where to travel to next.
   */
  struct goal_data : public rcppsw::patterns::state_machine::event_data {
    goal_data(argos::CVector2 loc_, double tolerance_) :
        tolerance(tolerance_), loc(loc_) {}
    goal_data(void) : tolerance(), loc() {}

    double tolerance;
    argos::CVector2 loc;
  };

  struct fsm_state {
    fsm_state(void) : last_collision_time(0) {}
    uint last_collision_time;
  };

  /* constants */

  /**
   * @brief The # of timesteps according to collision recovery. This is mainly
   * to ensure that you do not repeatedly get 2 robots butting heads as they try
   * to travel to opposite goals.
   */
  static uint kCOLLISION_RECOVERY_TIME;


  /* member functions */
  argos::CVector2 randomize_vector_angle(argos::CVector2 v);

  /**
   * @brief Calculates the relative vector from the robot to the current goal.
   *
   * @param goal The current goal.
   *
   * @return The vector, specified with the tail at the robot and the head
   * pointing towards the goal.
   */
  argos::CVector2 calc_vector_to_goal(const argos::CVector2& goal);

  /* states */
  FSM_STATE_DECLARE_ND(vector_fsm, start);
  FSM_STATE_DECLARE(vector_fsm, vector, struct goal_data);
  FSM_STATE_DECLARE_ND(vector_fsm, collision_avoidance);
  FSM_STATE_DECLARE_ND(vector_fsm, collision_recovery);
  FSM_STATE_DECLARE(vector_fsm, arrived, struct goal_data);

  FSM_ENTRY_DECLARE_ND(vector_fsm, entry_vector);
  FSM_ENTRY_DECLARE_ND(vector_fsm, entry_collision_avoidance);
  FSM_ENTRY_DECLARE_ND(vector_fsm, entry_collision_recovery);

  FSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
    FSM_DEFINE_STATE_MAP(state_map_ex, kSTATE_MAP) {
      FSM_STATE_MAP_ENTRY_EX_ALL(&start, NULL, NULL, NULL),
          FSM_STATE_MAP_ENTRY_EX_ALL(&vector, NULL, &entry_vector, NULL),
          FSM_STATE_MAP_ENTRY_EX_ALL(&collision_avoidance, NULL,
                                     &entry_collision_avoidance, NULL),
          FSM_STATE_MAP_ENTRY_EX_ALL(&collision_recovery, NULL,
                                     &entry_collision_recovery, NULL),
          FSM_STATE_MAP_ENTRY_EX_ALL(&arrived, NULL, NULL, NULL)
          };
    return &kSTATE_MAP[index];
  }

  vector_fsm(const vector_fsm& fsm) = delete;
  vector_fsm& operator=(const vector_fsm& fsm) = delete;

  argos::CRandom::CRNG* m_rng;
  struct fsm_state m_state;
  uint m_freq_collision_thresh;
  uint m_collision_rec_count;
  std::shared_ptr<controller::depth0::foraging_sensors> m_sensors;
  std::shared_ptr<controller::actuator_manager> m_actuators;
  struct goal_data m_goal_data;
  rcppsw::control::pid_loop m_ang_pid;
  rcppsw::control::pid_loop m_lin_pid;
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_VECTOR_FSM_HPP_ */
