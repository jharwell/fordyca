/**
 * @file stateless_controller.cpp
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
#include "fordyca/controller/depth0/stateless_controller.hpp"
#include <fstream>
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/base_sensing_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/fsm/depth0/stateless_fsm.hpp"
#include "rcppsw/robotics/hal/sensors/battery_sensor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateless_controller::stateless_controller(void)
    : base_controller(),
      ER_CLIENT_INIT("fordyca.controller.depth0.stateless"),
      m_fsm() {}

stateless_controller::~stateless_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stateless_controller::Init(ticpp::Element& node) {
  base_controller::Init(node);
  ndc_push();
  ER_INFO("Initializing...");
  m_fsm = rcppsw::make_unique<fsm::depth0::stateless_fsm>(
      base_controller::saa_subsystem());
  ER_INFO("Initialization finished");
  ndc_pop();
} /* Init() */

void stateless_controller::Reset(void) {
  base_controller::Reset();
  if (nullptr != m_fsm) {
    m_fsm->init();
  }
} /* Reset() */

void stateless_controller::ControlStep(void) {
  ndc_pusht();
  saa_subsystem()->actuation()->block_carry_throttle(is_carrying_block());
  saa_subsystem()->actuation()->throttling_update(
      saa_subsystem()->sensing()->tick());
  m_fsm->run();
  ndc_pop();
} /* ControlStep() */

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
FSM_WRAPPER_DEFINE_PTR(bool, stateless_controller, goal_acquired, m_fsm);

FSM_WRAPPER_DEFINE_PTR(acquisition_goal_type,
                       stateless_controller,
                       acquisition_goal,
                       m_fsm);

FSM_WRAPPER_DEFINE_PTR(transport_goal_type,
                       stateless_controller,
                       block_transport_goal,
                       m_fsm);

/*******************************************************************************
 * Distance Metrics
 ******************************************************************************/
__rcsw_pure double stateless_controller::distance(void) const {
  /*
   * If you allow distance gathering at timesteps < 1, you get a big jump
   * because of the prev/current location not being set up properly yet.
   */
  if (saa_subsystem()->sensing()->tick() > 1) {
    return saa_subsystem()->sensing()->heading().length();
  }
  return 0;
} /* distance() */

rmath::vector2d stateless_controller::velocity(void) const {
  /*
   * If you allow distance gathering at timesteps < 1, you get a big jump
   * because of the prev/current location not being set up properly yet.
   */
  if (saa_subsystem()->sensing()->tick() > 1) {
    return saa_subsystem()->linear_velocity();
  }
  return rmath::vector2d(0, 0);
} /* velocity() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_CONTROLLER(stateless_controller, "stateless_controller");
#pragma clang diagnostic pop

NS_END(depth0, controller, fordyca);
