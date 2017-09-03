/**
 * @file unpartitioned_task_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_UNPARTITIONED_TASK_FSM_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_UNPARTITIONED_TASK_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <utility>

#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>
#include "rcppsw/patterns/state_machine/hfsm.hpp"
#include "fordyca/params/params.hpp"
#include "fordyca/controller/sensor_manager.hpp"
#include "fordyca/controller/actuator_manager.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/controller/vector_to_goal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
namespace fsm = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class foraging_signal : public fsm::event_signal {
 public:
  enum {
    BLOCK_FOUND = fsm::event_signal::EXTERNAL_SIGNALS,
    ARRIVED_AT_TARGET
  };
};

class unpartitioned_task_fsm : public fsm::hfsm {
 public:
  unpartitioned_task_fsm(const struct foraging_fsm_params* params,
               const std::shared_ptr<rcppsw::common::er_server>& server,
               const std::shared_ptr<sensor_manager>& sensors,
               const std::shared_ptr<actuator_manager>& actuators,
               const std::shared_ptr<const representation::perceived_arena_map>& map);

  void init(void);

  uint8_t current_state(void) const { return m_current_state; }
  uint8_t max_states(void) const { return ST_MAX_STATES; }
  uint8_t previous_state(void) const { return m_previous_state; }

  bool is_searching_for_block(void) {
    return current_state() == ST_EXPLORE ||
        (current_state() == ST_LOCATE_BLOCK && !m_vector_fsm.in_progress());
  }
  void event_block_found(void);
  void run(void) { generated_event(true); state_engine(); }

 protected:
  enum fsm_states {
    ST_START,
    ST_EXPLORE,
    ST_NEW_DIRECTION,
    ST_RETURN_TO_NEST,
    ST_LEAVING_NEST,
    ST_COLLISION_AVOIDANCE,
    ST_LOCATE_BLOCK,
    ST_MAX_STATES
  };

 private:
  /* types */
  struct new_direction_data  : public fsm::event_data {
    explicit new_direction_data(argos::CRadians dir_) : dir(dir_) {}
    argos::CRadians dir;
  };
  struct fsm_state {
    fsm_state(void) : time_exploring_unsuccessfully(0) {}

    size_t time_exploring_unsuccessfully;
  };
  /* constants */

  /* member functions */
  argos::CVector2 randomize_vector_angle(argos::CVector2 vector);
  uint8_t next_state(void) const { return m_next_state; }
  uint8_t initial_state(void) const { return m_initial_state; }
  void next_state(uint8_t next_state) { m_next_state = next_state; }
  uint8_t last_state(void) const { return m_last_state; }

  void update_state(uint8_t update_state);
  bool acquire_block(void);
  void acquire_known_block(
      std::list<std::pair<const representation::block*, double>> blocks);

  /* non-hierarchical states */
  HFSM_STATE_DECLARE(unpartitioned_task_fsm, hfsm,
                     top_state, fsm::no_event_data,
                     start, fsm::no_event_data);
  HFSM_STATE_DECLARE(unpartitioned_task_fsm, hfsm,
                     top_state, fsm::no_event_data,
                     return_to_nest, fsm::no_event_data);
  HFSM_STATE_DECLARE(unpartitioned_task_fsm, hfsm,
                     top_state, fsm::no_event_data,
                     leaving_nest, fsm::no_event_data);

  HFSM_ENTRY_DECLARE(unpartitioned_task_fsm, entry_return_to_nest, fsm::no_event_data);
  HFSM_ENTRY_DECLARE(unpartitioned_task_fsm, entry_leaving_nest, fsm::no_event_data);
  HFSM_EXIT_DECLARE(unpartitioned_task_fsm, exit_leaving_nest);

  /*
   * States for locate_block sub-fsm. Note that the states for the
   * vector_to_goal sub-fsm cannot be part of the locate_block hfsm, because that
   * sub-fsm is initiated from multiple states, and hfsm states can only have
   * ONE parent state.
   **/
  HFSM_STATE_DECLARE(unpartitioned_task_fsm, hfsm,
                     top_state, fsm::no_event_data,
                     locate_block, fsm::event_data);
  /*
   * States for exploration sub-fsm (part of locate_block fsm).
   */
  HFSM_STATE_DECLARE(unpartitioned_task_fsm, unpartitioned_task_fsm,
                     locate_block, fsm::event_data,
                     explore, fsm::event_data);
  HFSM_STATE_DECLARE(unpartitioned_task_fsm, unpartitioned_task_fsm,
                     locate_block, fsm::event_data,
                     new_direction, new_direction_data);
  HFSM_STATE_DECLARE(unpartitioned_task_fsm, unpartitioned_task_fsm,
                     locate_block, fsm::event_data,
                     collision_avoidance, fsm::event_data);
  HFSM_ENTRY_DECLARE(unpartitioned_task_fsm, entry_new_direction, fsm::no_event_data);
  HFSM_ENTRY_DECLARE(unpartitioned_task_fsm, entry_explore, fsm::no_event_data);
  HFSM_ENTRY_DECLARE(unpartitioned_task_fsm, entry_collision_avoidance,
                     fsm::no_event_data);
  HFSM_EXIT_DECLARE(unpartitioned_task_fsm, exit_locate_block);

  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex) {
  HFSM_DEFINE_STATE_MAP(state_map_ex, kSTATE_MAP) {
        HFSM_STATE_MAP_ENTRY_EX_ALL(&start, NULL, NULL, NULL),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&explore, NULL, &entry_explore, NULL),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&new_direction, NULL,
                                   &entry_new_direction, NULL),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&return_to_nest, NULL,
                                   &entry_return_to_nest, NULL),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&leaving_nest, NULL,
                                   &entry_leaving_nest, &exit_leaving_nest),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&collision_avoidance, NULL,
                                   &entry_collision_avoidance, NULL),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&locate_block, NULL, NULL, &exit_locate_block)
    };
  HFSM_VERIFY_STATE_MAP(state_map_ex, kSTATE_MAP);
    return &kSTATE_MAP[0];
  }

  unpartitioned_task_fsm(const unpartitioned_task_fsm& fsm) = delete;
  unpartitioned_task_fsm& operator=(const unpartitioned_task_fsm& fsm) = delete;

  /* data members */
  uint8_t m_current_state;
  uint8_t m_next_state;
  uint8_t m_initial_state;
  uint8_t m_previous_state;
  uint8_t m_last_state;

  argos::CRandom::CRNG* m_rng;
  struct fsm_state m_state;
  std::shared_ptr<const struct foraging_fsm_params> mc_params;
  std::shared_ptr<sensor_manager> m_sensors;
  std::shared_ptr<actuator_manager> m_actuators;
  std::shared_ptr<const representation::perceived_arena_map> m_map;
  std::shared_ptr<rcppsw::common::er_server> m_server;
  vector_to_goal m_vector_fsm;
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_UNPARTITIONED_TASK_FSM_HPP_ */
