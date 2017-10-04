/**
 * @file base_foraging_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_BASE_FORAGING_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_BASE_FORAGING_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>
#include "rcsw/common/common.h"
#include "rcppsw/patterns/state_machine/hfsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace controller {
class sensor_manager;
class actuator_manager;
} /* namespace controller */

NS_START(fsm);
namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief The FSM for the most basic foraging definition: each robot executing
 * this FSM roams around randomly until it finds a block, and then brings the
 * block back to the nest and repeat.
 */
class base_foraging_fsm : public state_machine::hfsm {
 public:
  base_foraging_fsm(std::shared_ptr<rcppsw::common::er_server> server,
                    std::shared_ptr<controller::sensor_manager> sensors,
                    std::shared_ptr<controller::actuator_manager> actuators);

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
  argos::CVector2 randomize_vector_angle(argos::CVector2 vector);
  controller::actuator_manager*  actuators(void) const { return m_actuators.get(); }
  controller::sensor_manager*  sensors(void) const { return m_sensors.get(); }

  /* states */
  HFSM_STATE_DECLARE(base_foraging_fsm, return_to_nest, state_machine::no_event_data);
  HFSM_STATE_DECLARE(base_foraging_fsm, leaving_nest, state_machine::no_event_data);
  HFSM_STATE_DECLARE(base_foraging_fsm, collision_avoidance,
                     state_machine::no_event_data);

  HFSM_ENTRY_DECLARE(base_foraging_fsm, entry_return_to_nest,
                     state_machine::no_event_data);

  HFSM_ENTRY_DECLARE(base_foraging_fsm, entry_collision_avoidance,
                    state_machine::no_event_data);
  HFSM_ENTRY_DECLARE(base_foraging_fsm, entry_leaving_nest,
                     state_machine::no_event_data);

  base_foraging_fsm(const base_foraging_fsm& fsm) = delete;
  base_foraging_fsm& operator=(const base_foraging_fsm& fsm) = delete;

  argos::CRandom::CRNG* m_rng;
  std::shared_ptr<controller::sensor_manager> m_sensors;
  std::shared_ptr<controller::actuator_manager> m_actuators;
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_BASE_FORAGING_FSM_HPP_ */
