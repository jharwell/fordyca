/**
 * @file stateless_foraging_controller.cpp
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
#include "fordyca/controller/depth0/stateless_foraging_controller.hpp"
#include <fstream>
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/base_sensing_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/fsm/depth0/stateless_foraging_fsm.hpp"
#include "fordyca/params/depth0/stateless_controller_repository.hpp"
#include "rcppsw/robotics/hal/sensors/battery_sensor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateless_foraging_controller::stateless_foraging_controller(void)
    : base_foraging_controller(),
      ER_CLIENT_INIT("fordyca.controller.depth0.stateless"),
      m_fsm() {}

stateless_foraging_controller::~stateless_foraging_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stateless_foraging_controller::Init(ticpp::Element& node) {
  base_foraging_controller::Init(node);
  ndc_push();
  ER_INFO("Initializing...");
  params::depth0::stateless_controller_repository param_repo;
  param_repo.parse_all(node);

  if (!param_repo.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  m_fsm = rcppsw::make_unique<fsm::depth0::stateless_foraging_fsm>(
      base_foraging_controller::saa_subsystem());
  ER_INFO("Initialization finished");
  ndc_pop();
} /* Init() */

void stateless_foraging_controller::Reset(void) {
  base_foraging_controller::Reset();
  if (nullptr != m_fsm) {
    m_fsm->init();
  }
} /* Reset() */

void stateless_foraging_controller::ControlStep(void) {
  ndc_pusht();

  saa_subsystem()->actuation()->block_carry_throttle(is_carrying_block());
  saa_subsystem()->actuation()->throttling_update(
      saa_subsystem()->sensing()->tick());
  // add method for checking if the robot needs to get a cell from the arena.
      // check for robot's internal flag
  // add method for performing communication
  m_fsm->run();
  ndc_pop();
} /* ControlStep() */

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
FSM_WRAPPER_DEFINE_PTR(bool, stateless_foraging_controller, goal_acquired, m_fsm);

FSM_WRAPPER_DEFINE_PTR(acquisition_goal_type,
                       stateless_foraging_controller,
                       acquisition_goal,
                       m_fsm);

FSM_WRAPPER_DEFINE_PTR(transport_goal_type,
                       stateless_foraging_controller,
                       block_transport_goal,
                       m_fsm);

/*******************************************************************************
 * Distance Metrics
 ******************************************************************************/
__rcsw_pure double stateless_foraging_controller::distance(void) const {
  /*
   * If you allow distance gathering at timesteps < 1, you get a big jump
   * because of the prev/current location not being set up properly yet.
   */
  if (saa_subsystem()->sensing()->tick() > 1) {
    return saa_subsystem()->sensing()->heading().Length();
  }
  return 0;
} /* distance() */

argos::CVector2 stateless_foraging_controller::velocity(void) const {
  /*
   * If you allow distance gathering at timesteps < 1, you get a big jump
   * because of the prev/current location not being set up properly yet.
   */
  if (saa_subsystem()->sensing()->tick() > 1) {
    return saa_subsystem()->linear_velocity();
  }
  return argos::CVector2(0, 0);
} /* velocity() */

/* Notifiy ARGoS of the existence of the controller. */
using namespace argos;
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_CONTROLLER(stateless_foraging_controller,
                    "stateless_foraging_controller"); // NOLINT
#pragma clang diagnostic pop
NS_END(depth0, controller, fordyca);
