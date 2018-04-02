/**
 * @file actuation_subsystem.cpp
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
#include "fordyca/controller/actuation_subsystem.hpp"
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include "fordyca/params/actuation_params.hpp"
#include "rcppsw/er/server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
actuation_subsystem::actuation_subsystem(
    const struct params::actuation_params* c_params,
    struct actuator_list* const list,
    steering_force2D& steering)
    : mc_params(*c_params),
      m_actuators(*list),
      m_throttling(&c_params->throttling),
      m_fsm(&c_params->wheels, m_actuators.wheels, &m_throttling),
      m_steering(steering) {}

void actuation_subsystem::reset(void) {
  m_actuators.raba->ClearData();
  m_fsm.reset();
} /* reset() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void actuation_subsystem::set_rel_heading(const argos::CVector2& heading,
                                          bool force_hard_turn) {
  m_fsm.set_rel_heading(heading, force_hard_turn);
}

void actuation_subsystem::leds_set_color(const argos::CColor& color) {
  m_actuators.leds->SetAllColors(color);
} /* leds_set_color() */

NS_END(controller, fordyca);
