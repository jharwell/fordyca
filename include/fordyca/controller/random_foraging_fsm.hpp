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

#ifndef INCLUDE_FORDYCA_CONTROLLER_RANDOM_FORAGING_FSM_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_RANDOM_FORAGING_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>
#include "rcsw/common/common.h"
#include "rcppsw/patterns/state_machine/hfsm.hpp"
#include "fordyca/params/params.hpp"
#include "fordyca/controller/sensor_manager.hpp"
#include "fordyca/controller/actuator_manager.hpp"
#include "fordyca/controller/foraging_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
namespace fsm = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief The FSM for the most basic foraging definition: each robot executing
 * this FSM roams around randomly until it finds a block, and then brings the
 * block back to the nest and repeat.
 */
class random_foraging_fsm : public fsm::hfsm {
 public:
  random_foraging_fsm(const struct foraging_fsm_params* params,
                      std::shared_ptr<rcppsw::common::er_server> server,
                      std::shared_ptr<sensor_manager> sensors,
                      std::shared_ptr<actuator_manager> actuators);

  /**
   * @brief If TRUE the robot is roaming around looking for a block.
   */
  virtual bool is_exploring(void) const;

  /**
   * @brief If TRUE, the robot is returning to the nest, probably after having
   * successfully picked up a block.
   */
  virtual bool is_returning(void) const;

  /**
   * @brief If TRUE, the robot is currently engaged in collision avoidance.
   */
  virtual bool is_avoiding_collision(void) const;

  /**
   * @brief (Re)-initialize the FSM.
   */
  void init(void);

  /**
   * @brief Run the FSM in its current state, without injecting an event.
   */
  void run(void) { generated_event(true); state_engine(); }

 protected:
  /**
   * @brief Inject randomness into robot exploring by having them change their
   * direction every X timesteps if they have not yet located a block, where X
   * is set in the .argos configuration file.
   */
  struct new_direction_data : public fsm::event_data {
    explicit new_direction_data(argos::CRadians dir_) : dir(dir_) {}
    argos::CRadians dir;
  };

  struct fsm_state {
    fsm_state(void) : time_exploring_unsuccessfully(0),
                      last_collision_time(0) {}

    size_t time_exploring_unsuccessfully;
    uint last_collision_time;
  };

  enum fsm_states {
    ST_START,                 /* Initial state */
    ST_EXPLORE,               /* No known blocks--roam around looking for one  */
    ST_NEW_DIRECTION,         /* Time to change direction during exploration */
    ST_RETURN_TO_NEST,        /* Block found--bring it back to the nest */
    ST_LEAVING_NEST,          /* Block dropped in nest--time to go */
    ST_COLLISION_AVOIDANCE,   /* Avoiding colliding with something */
    ST_MAX_STATES
  };

  argos::CVector2 randomize_vector_angle(argos::CVector2 vector);

  /**
   * @brief Reset the # of timesteps the robot has spent unsuccessfully looking
   * for a block.
   */
  void explore_time_reset(void) { m_state.time_exploring_unsuccessfully = 0; }

  /**
   * @brief Increment the # of timesteps the robot has spent unsuccessfully
   * looking for a block.
   */
  void explore_time_inc(void) { ++m_state.time_exploring_unsuccessfully; }

  /**
   * @brief Get the # of timesteps the robot has spent unsuccessfully looking
   * for a block.
   */
  size_t explore_time(void) const { return m_state.time_exploring_unsuccessfully; }

  /* states */
  HFSM_STATE_DECLARE(random_foraging_fsm, start, fsm::no_event_data);
  HFSM_STATE_DECLARE(random_foraging_fsm, explore, fsm::event_data);
  HFSM_STATE_DECLARE(random_foraging_fsm, new_direction, fsm::event_data);
  HFSM_STATE_DECLARE(random_foraging_fsm, return_to_nest, fsm::no_event_data);
  HFSM_STATE_DECLARE(random_foraging_fsm, leaving_nest, fsm::no_event_data);
  HFSM_STATE_DECLARE(random_foraging_fsm, collision_avoidance,
                     fsm::no_event_data);

  HFSM_ENTRY_DECLARE(random_foraging_fsm, entry_explore,
                     fsm::no_event_data);
  HFSM_ENTRY_DECLARE(random_foraging_fsm, entry_new_direction,
                     fsm::no_event_data);
  HFSM_ENTRY_DECLARE(random_foraging_fsm, entry_return_to_nest,
                     fsm::no_event_data);

  HFSM_ENTRY_DECLARE(random_foraging_fsm, entry_collision_avoidance,
                    fsm::no_event_data);
  HFSM_ENTRY_DECLARE(random_foraging_fsm, entry_leaving_nest,
                     fsm::no_event_data);
  HFSM_EXIT_DECLARE(random_foraging_fsm, exit_leaving_nest);

  /* member functions */
  uint8_t current_state(void) const { return m_current_state; }
  uint8_t max_states(void) const { return ST_MAX_STATES; }
  uint8_t next_state(void) const { return m_next_state; }
  uint8_t initial_state(void) const { return m_initial_state; }
  void next_state(uint8_t next_state) { m_next_state = next_state; }
  uint8_t last_state(void) const { return m_last_state; }
  void update_state(uint8_t update_state);

  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) {
  HFSM_DEFINE_STATE_MAP(state_map_ex, kSTATE_MAP) {
    HFSM_STATE_MAP_ENTRY_EX(&start, hfsm::top_state()),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&explore, hfsm::top_state(),
                                    NULL,
                                    &entry_explore, NULL),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&new_direction, hfsm::top_state(),
                                    NULL,
                                   &entry_new_direction, NULL),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&return_to_nest, hfsm::top_state(),
                                    NULL,
                                   &entry_return_to_nest, NULL),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&leaving_nest, hfsm::top_state(),
                                    NULL,
                                   &entry_leaving_nest, &exit_leaving_nest),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&collision_avoidance, hfsm::top_state(),
                                    NULL,
                                    &entry_collision_avoidance, NULL),
    };
  HFSM_VERIFY_STATE_MAP(state_map_ex, kSTATE_MAP);
  return (&kSTATE_MAP[index]);
  }

  random_foraging_fsm(const random_foraging_fsm& fsm) = delete;
  random_foraging_fsm& operator=(const random_foraging_fsm& fsm) = delete;

  /**
   * How many timesteps to spend on a particular vector when exploring before
   * changing.
   */
  const double mc_unsuccessful_explore_dir_change;

  uint8_t m_current_state;
  uint8_t m_next_state;
  uint8_t m_initial_state;
  uint8_t m_previous_state;
  uint8_t m_last_state;

  argos::CRandom::CRNG* m_rng;
  argos::CRadians m_new_dir;
  struct fsm_state m_state;
  std::shared_ptr<sensor_manager> m_sensors;
  std::shared_ptr<actuator_manager> m_actuators;
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_RANDOM_FORAGING_FSM_HPP_ */
