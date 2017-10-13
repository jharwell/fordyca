/**
 * @file random_foraging_controller.cpp
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
#include "fordyca/controller/random_foraging_controller.hpp"
#include "fordyca/params/random_foraging_repository.hpp"
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/params/sensor_params.hpp"
#include "fordyca/params/actuator_params.hpp"
#include "fordyca/params/fsm_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
random_foraging_controller::random_foraging_controller(void) :
    m_fsm() {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void random_foraging_controller::Init(argos::TConfigurationNode& node) {
  base_foraging_controller::Init(node);

  ER_NOM("Initializing random_foraging controller");

  params::random_foraging_repository param_repo;
  param_repo.parse_all(node);
  param_repo.show_all(er_client::server_handle()->log_stream());

  m_fsm.reset(
      new fsm::random_foraging_fsm(static_cast<const struct params::fsm_params*>(
          param_repo.get_params("fsm")),
                                   base_foraging_controller::server(),
                                   base_foraging_controller::sensors(),
                                   base_foraging_controller::actuators()));
  ER_NOM("random_foraging controller initialization finished");
} /* Init() */

void random_foraging_controller::Reset(void) {
  m_fsm->init();
} /* Reset() */

/* Notifiy ARGoS of the existence of the controller. */
using namespace argos;
REGISTER_CONTROLLER(random_foraging_controller, "random_foraging_controller");

NS_END(controller, fordyca);
