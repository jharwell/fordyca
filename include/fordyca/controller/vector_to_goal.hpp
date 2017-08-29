/**
 * @file vector_to_goal.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_VECTOR_TO_GOAL_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_VECTOR_TO_GOAL_HPP_

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
class vector_to_goal : public fsm::simple_fsm {
 public:
  vector_to_goal(const struct foraging_fsm_params* params,
             std::shared_ptr<rcppsw::common::er_server> server,
             std::shared_ptr<sensor_manager> sensors,
             std::shared_ptr<actuator_manager> actuators);

  void init(void);
  bool arrived_at_goal(void) {return current_state() == ST_ARRIVED; }
  void run(void) { generated_event(true); state_engine(); }
  void event_start(const argos::CVector2& goal);

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

  struct goal_data : public fsm::event_data {
    explicit goal_data(argos::CVector2 goal_) : goal(goal_) {}
    goal_data(void) : goal() {}

    argos::CVector2 goal;
  };

  struct fsm_state {
    fsm_state(void) : time_exploring_unsuccessfully(0),
                      last_collision_time(0) {}

    size_t time_exploring_unsuccessfully;
    uint last_collision_time;
  };

  /* constants */
  static int kCOLLISION_RECOVERY_TIME;
  static double kVECTOR_TO_GOAL_MIN_DIFF;

  /* member functions */
  argos::CVector2 randomize_vector_angle(argos::CVector2 vector);

  /* states */
  FSM_STATE_DECLARE(vector_to_goal, start, fsm::no_event_data);
  FSM_STATE_DECLARE(vector_to_goal, vector, goal_data);
  FSM_STATE_DECLARE(vector_to_goal, collision_avoidance, fsm::no_event_data);
  FSM_STATE_DECLARE(vector_to_goal, collision_recovery, fsm::no_event_data);
  FSM_STATE_DECLARE(vector_to_goal, arrived, fsm::no_event_data);

  FSM_ENTRY_DECLARE(vector_to_goal, entry_vector,
                    fsm::no_event_data);
  FSM_ENTRY_DECLARE(vector_to_goal, entry_collision_avoidance,
                    fsm::no_event_data);
  FSM_ENTRY_DECLARE(vector_to_goal, entry_collision_recovery,
                    fsm::no_event_data);

  FSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex) {
  FSM_DEFINE_STATE_MAP(state_map_ex, kSTATE_MAP) {
        FSM_STATE_MAP_ENTRY_EX_ALL(&start, NULL, NULL, NULL),
        FSM_STATE_MAP_ENTRY_EX_ALL(&vector, NULL, &entry_vector, NULL),
        FSM_STATE_MAP_ENTRY_EX_ALL(&collision_avoidance, NULL,
                                   &entry_collision_avoidance, NULL),
        FSM_STATE_MAP_ENTRY_EX_ALL(&collision_recovery, NULL,
                                   &entry_collision_recovery, NULL),
        FSM_STATE_MAP_ENTRY_EX_ALL(&arrived, NULL, NULL, NULL),
    };
  FSM_VERIFY_STATE_MAP(state_map_ex, kSTATE_MAP);
    return &kSTATE_MAP[0];
  }

  vector_to_goal(const vector_to_goal& fsm) = delete;
  vector_to_goal& operator=(const vector_to_goal& fsm) = delete;

  /* data members */
  argos::CRandom::CRNG* m_rng;
  struct fsm_state m_state;
  uint m_freq_collision_thresh;
  std::shared_ptr<sensor_manager> m_sensors;
  std::shared_ptr<actuator_manager> m_actuators;
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_VECTOR_TO_GOAL_HPP_ */
