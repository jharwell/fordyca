/**
 * @file base_foraging_fsm.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/base_foraging_fsm.hpp"
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/controller/base_foraging_sensor_manager.hpp"
#include "fordyca/controller/actuator_manager.hpp"
#include "fordyca/controller/foraging_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_foraging_fsm::base_foraging_fsm(
    std::shared_ptr<rcppsw::common::er_server> server,
    std::shared_ptr<controller::base_foraging_sensor_manager> sensors,
    std::shared_ptr<controller::actuator_manager> actuators,
    uint8_t max_states) :
    state_machine::hfsm(server, max_states),
    HFSM_CONSTRUCT_STATE(return_to_nest, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(leaving_nest, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(collision_avoidance, hfsm::top_state()),
    entry_return_to_nest(),
    entry_collision_avoidance(),
    entry_leaving_nest(),
    m_rng(argos::CRandom::CreateRNG("argos")),
    m_sensors(sensors),
    m_actuators(actuators) {
  /* er_client::insmod("base_foraging_fsm", */
  /*                   rcppsw::common::er_lvl::DIAG, */
  /*                   rcppsw::common::er_lvl::NOM); */
}

/*******************************************************************************
 * States
 ******************************************************************************/
HFSM_STATE_DEFINE(base_foraging_fsm, leaving_nest, state_machine::event_data) {
  ER_ASSERT(state_machine::event_type::NORMAL == data->type(),
            "FATAL: ST_LEAVING_NEST cannot handle child events");
  ER_ASSERT(controller::foraging_signal::BLOCK_PICKUP != data->signal(),
            "FATAL: ST_LEAVING_NEST should never pickup blocks...");
  ER_ASSERT(controller::foraging_signal::BLOCK_DROP != data->signal(),
            "FATAL: ST_LEAVING_NEST should never drop blocks...");
  ER_ASSERT(controller::foraging_signal::BLOCK_LOCATED != data->signal(),
            "FATAL: ST_LEAVING_NEST should never locate blocks...");

  if (current_state() != last_state()) {
    ER_DIAG("Executing ST_LEAVING_NEST");
  }

  /*
   * The vector returned by calc_vector_to_light() points to the light. Thus,
   * the minus sign is because we want to go away from the light.
   */
  argos::CVector2 diff_vector;
  argos::CRadians current_heading = m_sensors->calc_vector_to_light().Angle();
  m_sensors->calc_diffusion_vector(&diff_vector);
  m_actuators->set_heading(m_actuators->max_wheel_speed() * diff_vector -
                           argos::CVector2(m_actuators->max_wheel_speed() * 0.25,
                                           current_heading));
  if (!m_sensors->in_nest()) {
    return controller::foraging_signal::LEFT_NEST;
  }
  return state_machine::event_signal::HANDLED;
}
HFSM_STATE_DEFINE(base_foraging_fsm, return_to_nest, state_machine::event_data) {
  ER_ASSERT(state_machine::event_type::NORMAL == data->type(),
            "FATAL: ST_RETURN_TO_NEST cannot handle child events");
  ER_ASSERT(controller::foraging_signal::BLOCK_PICKUP != data->signal(),
            "FATAL: ST_RETURN_TO_NEST should never pickup blocks...");
  ER_ASSERT(controller::foraging_signal::BLOCK_LOCATED != data->signal(),
            "FATAL: ST_RETURN_TO_NEST should never locate blocks...");

  if (current_state() != last_state()) {
    ER_DIAG("Executing ST_RETURN_TO_NEST");
  }

  /*
   * We have arrived at the nest and it's time to head back out again. The
   * loop functions need to call the drop_block() function, as they have to
   * redistribute it (the FSM has no idea how to do that).
   *
   * The BLOCK_DROP signal comes from the loop functions the same timestep as we
   * realize that we are in the nest, so we need to be sure that we return the
   * BLOCK_DROP signal to the upper FSM, as that it what it is listening for.
   */
  if (controller::foraging_signal::BLOCK_DROP == data->signal()) {
    ER_ASSERT(m_sensors->in_nest(), "FATAL: BLOCK_DROP outside nest");
    return data->signal();
  }
  argos::CVector2 vector;

  /*
   * Check for nearby obstacles, and if so go into obstacle avoidance.
   */
  if (base_foraging_fsm::sensors()->calc_diffusion_vector(NULL)) {
    return controller::foraging_signal::COLLISION_IMMINENT;
  }

  /* ignore all obstacles for now... */
  m_sensors->calc_diffusion_vector(&vector);
  m_actuators->set_heading(m_actuators->max_wheel_speed() *
                           m_sensors->calc_vector_to_light());
  return state_machine::event_signal::HANDLED;
}
HFSM_STATE_DEFINE_ND(base_foraging_fsm, collision_avoidance) {
  argos::CVector2 vector;

  if (current_state() != last_state()) {
    ER_DIAG("Executing ST_COLLIISION_AVOIDANCE");
  }

  /* all signals ignored in this state */

  if (m_sensors->calc_diffusion_vector(&vector)) {
    m_actuators->set_heading(vector);
  } else {
    internal_event(previous_state());
  }
  return state_machine::event_signal::HANDLED;
}
HFSM_ENTRY_DEFINE_ND(base_foraging_fsm, entry_leaving_nest) {
  ER_DIAG("Entering ST_LEAVING_NEST");
  m_actuators->leds_set_color(argos::CColor::WHITE);
}
HFSM_ENTRY_DEFINE_ND(base_foraging_fsm, entry_return_to_nest) {
  ER_DIAG("Entering ST_RETURN_TO_NEST");
  m_actuators->leds_set_color(argos::CColor::GREEN);
}
HFSM_ENTRY_DEFINE_ND(base_foraging_fsm, entry_collision_avoidance) {
  ER_DIAG("Entering ST_COLLISION_AVOIDANCE");
  m_actuators->leds_set_color(argos::CColor::RED);
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void base_foraging_fsm::init(void) {
  m_actuators->reset();
  hfsm::init();
} /* init() */

argos::CVector2 base_foraging_fsm::randomize_vector_angle(argos::CVector2 vector) {
  argos::CRange<argos::CRadians> range(argos::CRadians(0.0),
                                       argos::CRadians(1.0));
  vector.Rotate(m_rng->Uniform(range));
  return vector;
} /* randomize_vector_angle() */

NS_END(controller, fordyca);
