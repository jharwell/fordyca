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
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/datatypes/color.h>
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/base_sensing_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/fsm/new_direction_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
namespace state_machine = rcppsw::patterns::state_machine;
using controller::steering_force_type;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_foraging_fsm::base_foraging_fsm(
    uint unsuccessful_dir_change_thresh,
    const std::shared_ptr<rcppsw::er::server>& server,
    std::shared_ptr<controller::saa_subsystem> saa,
    uint8_t max_states)
    : state_machine::hfsm(server, max_states),
      HFSM_CONSTRUCT_STATE(transport_to_nest, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(leaving_nest, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(collision_avoidance, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(new_direction, hfsm::top_state()),
      entry_transport_to_nest(),
      entry_collision_avoidance(),
      entry_leaving_nest(),
      entry_new_direction(),
      entry_wait_for_signal(),
      mc_dir_change_thresh(unsuccessful_dir_change_thresh),
      m_new_dir(),
      m_rng(argos::CRandom::CreateRNG("argos")),
      m_saa(std::move(saa)) {}

base_foraging_fsm::base_foraging_fsm(
    const std::shared_ptr<rcppsw::er::server>& server,
    const std::shared_ptr<controller::saa_subsystem>& saa,
    uint8_t max_states)
    : base_foraging_fsm(0, server, saa, max_states) {}

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

  if (current_state() != last_state()) {
    ER_DIAG("Executing ST_LEAVING_NEST");
  }

  m_saa->steering_force().anti_phototaxis();
  m_saa->actuation()->set_rel_heading(m_saa->steering_force().value());
  m_saa->steering_force().reset();

  if (!m_saa->sensing()->in_nest()) {
    return controller::foraging_signal::LEFT_NEST;
  }
  return state_machine::event_signal::HANDLED;
}
HFSM_STATE_DEFINE(base_foraging_fsm,
                  transport_to_nest,
                  state_machine::event_data) {
  ER_ASSERT(state_machine::event_type::NORMAL == data->type(),
            "FATAL: ST_TRANSPORT_TO_NEST cannot handle child events");
  ER_ASSERT(controller::foraging_signal::BLOCK_PICKUP != data->signal(),
            "FATAL: ST_TRANSPORT_TO_NEST should never pickup blocks...");

  if (current_state() != last_state()) {
    ER_DIAG("Executing ST_TRANSPORT_TO_NEST");
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
    ER_ASSERT(m_saa->sensing()->in_nest(), "FATAL: BLOCK_DROP outside nest");
    return data->signal();
  }

  m_saa->steering_force().phototaxis();
  m_saa->steering_force().avoidance(
      m_saa->sensing()->threatening_obstacle_exists(),
      m_saa->sensing()->find_closest_obstacle());
  m_saa->actuation()->set_rel_heading(m_saa->steering_force().value());
  m_saa->steering_force().reset();
  return state_machine::event_signal::HANDLED;
}
HFSM_STATE_DEFINE_ND(base_foraging_fsm, collision_avoidance) {
  if (current_state() != last_state()) {
    ER_DIAG("Executing ST_COLLISION_AVOIDANCE");
  }

  if (m_saa->sensing()->threatening_obstacle_exists()) {
    m_saa->steering_force().avoidance(
        m_saa->sensing()->threatening_obstacle_exists(),
        m_saa->sensing()->find_closest_obstacle());
    m_saa->actuation()->set_rel_heading(m_saa->steering_force().value());

    argos::CVector2 force = m_saa->steering_force().value();
    m_saa->steering_force().reset();
    ER_VER("Still found threatening obstacle: avoidance force=(%f, %f)@%f [%f]",
           force.GetX(),
           force.GetY(),
           force.Angle().GetValue(),
           force.Length());
} else {
    internal_event(previous_state());
  }
  return state_machine::event_signal::HANDLED;
}

HFSM_STATE_DEFINE(base_foraging_fsm, new_direction, state_machine::event_data) {
  argos::CRadians current_dir = m_saa->sensing()->heading_angle();

  /*
   * The new direction is only passed the first time this state is entered, so
   * save it. After that, a standard HFSM signal is passed we which ignore.
   */
  auto* dir_data = dynamic_cast<const new_direction_data*>(data);
  if (nullptr != dir_data) {
    m_new_dir = dir_data->dir;
    m_new_dir_count = 0;
    ER_DIAG("Change direction: %f -> %f",
            current_dir.GetValue(),
            m_new_dir.GetValue());
  }

  /*
   * The amount we change our direction is proportional to how far off we are
   * from our desired new direction. This prevents excessive spinning due to
   * overshoot. See #191.
   */
  actuators()->set_rel_heading(
      argos::CVector2(
          base_foraging_fsm::actuators()->differential_drive().max_speed() *
              0.1,
          (current_dir - m_new_dir)),
      true);

  /*
   * We limit the maximum # of steps that we spin, and have an arrival tolerance
   * to also help limit excessive spinning. See #191.
   */
  if (std::fabs((current_dir - m_new_dir).GetValue()) < kDIR_CHANGE_TOL ||
      m_new_dir_count >= kDIR_CHANGE_MAX_STEPS) {
    internal_event(previous_state());
  }
  ++m_new_dir_count;
  return controller::foraging_signal::HANDLED;
}

HFSM_ENTRY_DEFINE_ND(base_foraging_fsm, entry_leaving_nest) {
  ER_DIAG("Entering ST_LEAVING_NEST");
  m_saa->actuation()->leds_set_color(argos::CColor::WHITE);
}
HFSM_ENTRY_DEFINE_ND(base_foraging_fsm, entry_transport_to_nest) {
  ER_DIAG("Entering ST_TRANSPORT_TO_NEST");
  m_saa->actuation()->leds_set_color(argos::CColor::GREEN);
}
HFSM_ENTRY_DEFINE_ND(base_foraging_fsm, entry_collision_avoidance) {
  ER_DIAG("Entering ST_COLLISION_AVOIDANCE");
  m_saa->actuation()->leds_set_color(argos::CColor::RED);
}
HFSM_ENTRY_DEFINE_ND(base_foraging_fsm, entry_new_direction) {
  actuators()->leds_set_color(argos::CColor::CYAN);
}
HFSM_ENTRY_DEFINE_ND(base_foraging_fsm, entry_wait_for_signal) {
  actuators()->leds_set_color(argos::CColor::WHITE);
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void base_foraging_fsm::init(void) {
  m_saa->actuation()->reset();
  hfsm::init();
} /* init() */

argos::CVector2 base_foraging_fsm::randomize_vector_angle(argos::CVector2 vector) {
  argos::CRange<argos::CRadians> range(argos::CRadians(0.0),
                                       argos::CRadians(1.0));
  vector.Rotate(m_rng->Uniform(range));
  return vector;
} /* randomize_vector_angle() */

const std::shared_ptr<const controller::base_sensing_subsystem> base_foraging_fsm::base_sensors(void) const {
  return m_saa->sensing();
} /* base_sensors() */

const std::shared_ptr<controller::base_sensing_subsystem> base_foraging_fsm::base_sensors(void) {
  return m_saa->sensing();
} /* base_actuation() */

const std::shared_ptr<const controller::actuation_subsystem> base_foraging_fsm::actuators(void) const {
  return m_saa->actuation();
} /* actuators() */

const std::shared_ptr<controller::actuation_subsystem> base_foraging_fsm::actuators(void) {
  return m_saa->actuation();
} /* actuators() */

NS_END(controller, fordyca);
