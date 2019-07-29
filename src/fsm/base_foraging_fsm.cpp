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
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/fsm/new_direction_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_foraging_fsm::base_foraging_fsm(controller::saa_subsystem* const saa,
                                     uint8_t max_states)
    : rpfsm::hfsm(max_states),
      ER_CLIENT_INIT("fordyca.fsm.base_foraging"),
      HFSM_CONSTRUCT_STATE(transport_to_nest, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(leaving_nest, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(new_direction, hfsm::top_state()),
      m_rng(argos::CRandom::CreateRNG("argos")),
      m_saa(saa),
      m_tracker(m_saa) {}

/*******************************************************************************
 * States
 ******************************************************************************/
HFSM_STATE_DEFINE(base_foraging_fsm, leaving_nest, rpfsm::event_data* data) {
  ER_ASSERT(rpfsm::event_type::ekNORMAL == data->type(),
            "ekST_LEAVING_NEST cannot handle child events");
  ER_ASSERT(controller::foraging_signal::ekBLOCK_PICKUP != data->signal(),
            "ekST_LEAVING_NEST should never pickup blocks...");
  ER_ASSERT(controller::foraging_signal::ekBLOCK_DROP != data->signal(),
            "ekST_LEAVING_NEST should never drop blocks...");

  if (current_state() != last_state()) {
    ER_DEBUG("Executing ekST_LEAVING_NEST");
  }
  /*
   * We don't want to just apply anti-phototaxis force, because that will make
   * the robot immediately turn around as soon as it has entered the nest and
   * dropped its block, leading to a lot of traffic jams by the edge of the
   * nest. Instead, wander about within the nest until you find the edge (either
   * on your own or being pushed out via collision avoidance).
   */
  if (auto obs = m_saa->sensing()->avg_obstacle_within_prox()) {
    m_tracker.ca_enter();
    saa_subsystem()->steer2D_force_calc().avoidance(*obs);
  } else {
    m_tracker.ca_exit();
  }
  saa_subsystem()->steer2D_force_calc().wander();
  m_saa->steer2D_force_apply();

  if (!m_saa->sensing()->in_nest()) {
    return controller::foraging_signal::ekLEFT_NEST;
  }
  return rpfsm::event_signal::ekHANDLED;
} /* leaving_nest() */

HFSM_STATE_DEFINE(base_foraging_fsm, transport_to_nest, rpfsm::event_data* data) {
  ER_ASSERT(rpfsm::event_type::ekNORMAL == data->type(),
            "ekST_TRANSPORT_TO_NEST cannot handle child events");
  ER_ASSERT(controller::foraging_signal::ekBLOCK_PICKUP != data->signal(),
            "ekST_TRANSPORT_TO_NEST should never pickup blocks...");
  ER_ASSERT(controller::foraging_signal::ekBLOCK_PICKUP != data->signal(),
            "ekST_TRANSPORT_TO_NEST should never drop blocks");
  if (current_state() != last_state()) {
    ER_DEBUG("Executing ekST_TRANSPORT_TO_NEST");
  }

  /*
   * We have arrived at the nest so send this signal to the parent FSM that is
   * listing for it.
   */
  if (m_saa->sensing()->in_nest()) {
    if (m_nest_count++ < kNEST_COUNT_MAX_STEPS) {
      m_saa->steer2D_force_calc().wander();
      m_saa->steer2D_force_apply();
      return controller::foraging_signal::ekHANDLED;
    } else {
      m_nest_count = 0;
      return controller::foraging_signal::ekENTERED_NEST;
    }
  }

  /*
   * Add a bit of wander force when returning to the nest so that we do not
   * beeline for its center directly to decrease congestion.
   */
  m_saa->steer2D_force_calc().wander();
  m_saa->steer2D_force_calc().value(m_saa->steer2D_force_calc().value() * 0.5);
  m_saa->steer2D_force_calc().phototaxis(m_saa->sensing()->light().readings());

  if (auto obs = m_saa->sensing()->avg_obstacle_within_prox()) {
    m_tracker.ca_enter();
    m_saa->steer2D_force_calc().avoidance(*obs);
  } else {
    m_tracker.ca_exit();
  }

  m_saa->steer2D_force_apply();
  return rpfsm::event_signal::ekHANDLED;
} /* transport_to_nest() */

HFSM_STATE_DEFINE(base_foraging_fsm, new_direction, rpfsm::event_data* data) {
  rmath::radians current_dir = m_saa->sensing()->heading();

  /*
   * The new direction is only passed the first time this state is entered, so
   * save it. After that, a standard HFSM signal is passed we which ignore.
   */
  auto* dir_data = dynamic_cast<const new_direction_data*>(data);
  if (nullptr != dir_data) {
    m_new_dir = dir_data->dir;
    m_new_dir_count = 0;
    ER_DEBUG("Change direction: %f -> %f",
             current_dir.value(),
             m_new_dir.value());
  }

  /*
   * The amount we change our direction is proportional to how far off we are
   * from our desired new direction. This prevents excessive spinning due to
   * overshoot. See #191.
   */
  actuators()->differential_drive().fsm_drive(
      base_foraging_fsm::actuators()->differential_drive().max_speed() * 0.1,
      (current_dir - m_new_dir));

  /*
   * We limit the maximum # of steps that we spin, and have an arrival tolerance
   * to also help limit excessive spinning. See #191.
   */
  if (std::fabs((current_dir - m_new_dir).value()) < kDIR_CHANGE_TOL ||
      m_new_dir_count >= kDIR_CHANGE_MAX_STEPS) {
    internal_event(previous_state());
  }
  ++m_new_dir_count;
  return controller::foraging_signal::ekHANDLED;
}

HFSM_ENTRY_DEFINE_ND(base_foraging_fsm, entry_leaving_nest) {
  m_saa->actuation()->leds_set_color(rutils::color::kWHITE);
}
HFSM_ENTRY_DEFINE_ND(base_foraging_fsm, entry_transport_to_nest) {
  m_saa->sensing()->light().enable();
  m_saa->actuation()->leds_set_color(rutils::color::kGREEN);
}
HFSM_EXIT_DEFINE(base_foraging_fsm, exit_transport_to_nest) {
  m_saa->sensing()->light().disable();
}
HFSM_ENTRY_DEFINE_ND(base_foraging_fsm, entry_new_direction) {
  actuators()->leds_set_color(rutils::color::kCYAN);
}
HFSM_ENTRY_DEFINE_ND(base_foraging_fsm, entry_wait_for_signal) {
  actuators()->differential_drive().stop();
  actuators()->leds_set_color(rutils::color::kWHITE);
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void base_foraging_fsm::init(void) {
  m_saa->actuation()->reset();
  hfsm::init();
} /* init() */

rmath::vector2d base_foraging_fsm::randomize_vector_angle(
    const rmath::vector2d& vector) {
  argos::CRange<argos::CRadians> range(argos::CRadians(0.0),
                                       argos::CRadians(1.0));
  argos::CVector2 tmp(vector.x(), vector.y());
  tmp.Rotate(m_rng->Uniform(range));
  return {tmp.GetX(), tmp.GetY()};
} /* randomize_vector_angle() */

const std::shared_ptr<const controller::sensing_subsystem> base_foraging_fsm::sensors(
    void) const {
  return m_saa->sensing();
} /* base_sensors() */

const std::shared_ptr<controller::sensing_subsystem> base_foraging_fsm::sensors(
    void) {
  return m_saa->sensing();
} /* base_actuation() */

const std::shared_ptr<const controller::actuation_subsystem> base_foraging_fsm::
    actuators(void) const {
  return m_saa->actuation();
} /* actuators() */

const std::shared_ptr<controller::actuation_subsystem> base_foraging_fsm::actuators(
    void) {
  return m_saa->actuation();
} /* actuators() */

NS_END(controller, fordyca);
