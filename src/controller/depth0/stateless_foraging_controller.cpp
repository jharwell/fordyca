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

#include "fordyca/controller/actuator_manager.hpp"
#include "fordyca/controller/base_foraging_sensors.hpp"
#include "fordyca/fsm/depth0/stateless_foraging_fsm.hpp"
#include "fordyca/params/actuator_params.hpp"
#include "fordyca/params/depth0/stateless_foraging_repository.hpp"
#include "fordyca/params/fsm_params.hpp"
#include "fordyca/params/sensor_params.hpp"
#include "fordyca/representation/line_of_sight.hpp"
#include "rcppsw/er/server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateless_foraging_controller::stateless_foraging_controller(void)
    : base_foraging_controller(), m_fsm() {}

stateless_foraging_controller::~stateless_foraging_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stateless_foraging_controller::Init(argos::TConfigurationNode& node) {
  base_foraging_controller::Init(node);

  ER_NOM("Initializing stateless_foraging controller");

  params::depth0::stateless_foraging_repository param_repo;
  param_repo.parse_all(node);
  param_repo.show_all(client::server_handle()->log_stream());
  ER_ASSERT(param_repo.validate_all(),
            "FATAL: Not all parameters were validated");

  m_fsm = rcppsw::make_unique<fsm::depth0::stateless_foraging_fsm>(
      static_cast<const struct params::fsm_params*>(
          param_repo.get_params("fsm")),
      base_foraging_controller::server(),
      base_foraging_controller::base_sensors_ref(),
      base_foraging_controller::actuators());
  ER_NOM("stateless_foraging controller initialization finished");
} /* Init() */

void stateless_foraging_controller::Reset(void) {
  base_foraging_controller::Reset();
  if (nullptr != m_fsm) {
    m_fsm->init();
  }
} /* Reset() */

void stateless_foraging_controller::ControlStep(void) {
  if (is_carrying_block()) {
    actuators()->set_speed_throttle(true);
  } else {
    actuators()->set_speed_throttle(false);
  }
  m_fsm->run();
} /* ControlStep() */

bool stateless_foraging_controller::block_acquired(void) const {
  return m_fsm->block_acquired();
} /* block_acquired() */

bool stateless_foraging_controller::is_transporting_to_nest(void) const {
  return m_fsm->is_transporting_to_nest();
} /* is_transporting_to_nest() */

/*******************************************************************************
 * Distance Metrics
 ******************************************************************************/
int stateless_foraging_controller::entity_id(void) const {
  return std::atoi(GetId().c_str() + 2);
} /* entity_id() */

__pure double stateless_foraging_controller::timestep_distance(void) const {
  /*
   * If you allow distance gathering at timesteps < 1, you get a big jump
   * because of the prev/current location not being set up properly yet.
   */
  if (base_sensors()->tick() > 1) {
    return base_sensors()->robot_heading().Length();
  }
  return 0;
} /* timestep_distance() */

/* Notifiy ARGoS of the existence of the controller. */
using namespace argos;
REGISTER_CONTROLLER(stateless_foraging_controller,
                    "stateless_foraging_controller"); // NOLINT

NS_END(depth0, controller, fordyca);
